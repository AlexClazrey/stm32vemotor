#include "can_io.h"

// reference to def in main.c
extern CAN_HandleTypeDef hcan;

uint32_t can_tx_mailbox_used = 0;
static uint16_t can_selfid = 0;
static CAN_CmdListener cmdlis;
static CAN_ReplyListener replis;


void can_init(uint16_t self_id) {
	can_set_id(self_id);
	HAL_CAN_Start(&hcan);
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
}
void can_set_cmdlistener(CAN_CmdListener lis) {
	cmdlis = lis;
}
void can_set_replylistener(CAN_ReplyListener lis) {
	replis = lis;
}

#define CMD_HEADER 0x000
#define BROADCAST_HEADER 0x100
#define REPLY_OK_HEADER 0x700
#define REPLY_BAD_HEADER 0x300

// selfid 的低位字节装在 stdid 里面，高位字节装在报文的第一个字节里。
static uint32_t make_packet_id(uint32_t header, uint16_t self_id, uint16_t to_id) {
	return ((header + (self_id & 0xFF)) << 18) + to_id;
}

void can_set_id(uint16_t id) {
	can_selfid = id;

	// 移位的原理参照STM32 Reference Manual CAN Filter，中文版的432页。
	uint32_t cmdid = make_packet_id(CMD_HEADER, 0, id);
	// this filter has index 0 on FIFO0
	CAN_FilterTypeDef filter_cmd = {
		.FilterIdHigh = (cmdid >> 13),
		.FilterIdLow = (cmdid << 3) & 0xFFFF,
		.FilterMaskIdHigh = 0xE007,
		.FilterMaskIdLow = 0xFFF8,
		.FilterFIFOAssignment = CAN_FILTER_FIFO0,
		.FilterBank = 0,
		.FilterMode = CAN_FILTERMODE_IDMASK,
		.FilterScale = CAN_FILTERSCALE_32BIT,
		.FilterActivation = CAN_FILTER_ENABLE,
	};
	HAL_CAN_ConfigFilter(&hcan, &filter_cmd);

	// this filter has index 1 on FIFO0
	CAN_FilterTypeDef filter_broadcast = {
		.FilterIdHigh = (BROADCAST_HEADER) << 5,
		.FilterIdLow = 0,
		.FilterMaskIdHigh = 0xE000,
		.FilterMaskIdLow = 0,
		.FilterFIFOAssignment = CAN_FILTER_FIFO0,
		.FilterBank = 1,
		.FilterMode = CAN_FILTERMODE_IDMASK,
		.FilterScale = CAN_FILTERSCALE_32BIT,
		.FilterActivation = CAN_FILTER_ENABLE,
	};
	HAL_CAN_ConfigFilter(&hcan, &filter_broadcast);

	// this filter has index 0 on FIFO1
	uint32_t replyokid = make_packet_id(REPLY_OK_HEADER, 0, id);
	CAN_FilterTypeDef filter_reply_ok = {
		.FilterIdHigh = (replyokid >> 13),
		.FilterIdLow = (replyokid << 3) & 0xFFFF,
		.FilterMaskIdHigh = 0xE007,
		.FilterMaskIdLow = 0xFFF8,
		.FilterFIFOAssignment = CAN_FILTER_FIFO1,
		.FilterBank = 2,
		.FilterMode = CAN_FILTERMODE_IDMASK,
		.FilterScale = CAN_FILTERSCALE_32BIT,
		.FilterActivation = CAN_FILTER_ENABLE,
	};
	HAL_CAN_ConfigFilter(&hcan, &filter_reply_ok);

	uint32_t replybadid = make_packet_id(REPLY_BAD_HEADER, 0, id);
	CAN_FilterTypeDef filter_reply_bad = {
		.FilterIdHigh = (replybadid >> 13),
		.FilterIdLow = (replybadid << 3) & 0xFFFF,
		.FilterMaskIdHigh = 0xE007,
		.FilterMaskIdLow = 0xFFF8,
		.FilterFIFOAssignment = CAN_FILTER_FIFO1,
		.FilterBank = 3,
		.FilterMode = CAN_FILTERMODE_IDMASK,
		.FilterScale = CAN_FILTERSCALE_32BIT,
		.FilterActivation = CAN_FILTER_ENABLE,
	};
	HAL_CAN_ConfigFilter(&hcan, &filter_reply_bad);
}

// data must have 8 bytes can be read.
HAL_StatusTypeDef can_send_cmd(uint8_t *data, uint8_t len, uint16_t to) {
	CAN_TxHeaderTypeDef txh = {0};
	if(to == 0) {
		txh.ExtId = make_packet_id(BROADCAST_HEADER, can_selfid, 0);
	} else {
		txh.ExtId = make_packet_id(CMD_HEADER, can_selfid, to);
	}
	data[0] = can_selfid >> 8;
	if(len == 0) {
		len = 1;
	}
	txh.IDE = CAN_ID_EXT;
	txh.RTR = CAN_RTR_DATA;
	txh.DLC = len;
	txh.TransmitGlobalTime = DISABLE;
	return HAL_CAN_AddTxMessage(&hcan, &txh, data, &can_tx_mailbox_used);
}

HAL_StatusTypeDef can_send_reply(uint8_t to, _Bool ok) {
	static uint8_t data[8] = {0};
	CAN_TxHeaderTypeDef txh = {0};
	txh.ExtId = make_packet_id(ok ? REPLY_OK_HEADER : REPLY_BAD_HEADER, can_selfid, to);
	data[0] = can_selfid >> 8;
	txh.IDE = CAN_ID_EXT;
	txh.RTR = CAN_RTR_DATA;
	txh.DLC = 1;
	txh.TransmitGlobalTime = DISABLE;
	return HAL_CAN_AddTxMessage(&hcan, &txh, data, &can_tx_mailbox_used);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *pcan) {
	CAN_RxHeaderTypeDef rxh = {0};
	uint8_t data[8] = {0};
	if (HAL_CAN_GetRxMessage(pcan, CAN_RX_FIFO0, &rxh, data) == HAL_OK) {
		uint16_t from = ((rxh.ExtId >> 18) & 0xFF) + (data[0] << 8);
		if(cmdlis) {
			cmdlis(data, rxh.DLC, from, rxh.FilterMatchIndex == 1);
		}
	}
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *pcan) {
	CAN_RxHeaderTypeDef rxh = {0};
	uint8_t data[8] = {0};
	if (HAL_CAN_GetRxMessage(pcan, CAN_RX_FIFO1, &rxh, data) == HAL_OK) {
		uint16_t from = ((rxh.ExtId >> 18) & 0xFF) + (data[0] << 8);
		_Bool isok = ((rxh.ExtId >> 18) & REPLY_OK_HEADER) == REPLY_OK_HEADER;
		if(replis) {
			replis(isok, from);
		}
	}
}
