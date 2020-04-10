#include "can_io.h"

// reference to def in main.c
extern CAN_HandleTypeDef hcan;

uint32_t can_tx_mailbox_used = 0;
static uint8_t can_selfid = 0;
static CAN_CmdListener cmdlis;
static CAN_ReplyListener replis;


void can_init(uint8_t selfid) {
	can_set_id(selfid);
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
#define REPLY_OK_HEADER 0x700
#define REPLY_BAD_HEADER 0x300
void can_set_id(uint8_t id) {
	can_selfid = id;
	CAN_FilterTypeDef filter1 = {
		.FilterIdHigh = (CMD_HEADER + id) << 5,
		.FilterIdLow = 0,
		.FilterMaskIdHigh = 0xFFE0,
		.FilterMaskIdLow = 0,
		.FilterFIFOAssignment = CAN_FILTER_FIFO0,
		.FilterBank = 0,
		.FilterMode = CAN_FILTERMODE_IDMASK,
		.FilterScale = CAN_FILTERSCALE_32BIT,
		.FilterActivation = CAN_FILTER_ENABLE,
	};
	HAL_CAN_ConfigFilter(&hcan, &filter1);
	CAN_FilterTypeDef filter2 = {
		.FilterIdHigh = (REPLY_OK_HEADER + id) << 5,
		.FilterIdLow = (REPLY_BAD_HEADER + id) << 5,
		.FilterMaskIdHigh = 0xFFE0,
		.FilterMaskIdLow = 0xFFE0,
		.FilterFIFOAssignment = CAN_FILTER_FIFO1,
		.FilterBank = 1,
		.FilterMode = CAN_FILTERMODE_IDMASK,
		.FilterScale = CAN_FILTERSCALE_16BIT,
		.FilterActivation = CAN_FILTER_ENABLE,
	};
	HAL_CAN_ConfigFilter(&hcan, &filter2);
}

// data must have 8 bytes can be read.
HAL_StatusTypeDef can_send_cmd(uint8_t *data, uint8_t len, uint8_t to) {
	CAN_TxHeaderTypeDef txh = {0};
	txh.StdId = CMD_HEADER + to;
	txh.ExtId = (txh.StdId << 18) + can_selfid;
	txh.IDE = CAN_ID_EXT;
	txh.RTR = CAN_RTR_DATA;
	txh.DLC = len;
	txh.TransmitGlobalTime = DISABLE;
	return HAL_CAN_AddTxMessage(&hcan, &txh, data, &can_tx_mailbox_used);
}

HAL_StatusTypeDef can_send_reply(uint8_t to, _Bool ok) {
	static uint8_t emptydata[8] = {0};
	CAN_TxHeaderTypeDef txh = {0};
	txh.StdId = (ok ? REPLY_OK_HEADER : REPLY_BAD_HEADER) + to;
	txh.ExtId = (txh.StdId << 18) + can_selfid;
	txh.IDE = CAN_ID_EXT;
	txh.RTR = CAN_RTR_DATA;
	txh.DLC = 0;
	txh.TransmitGlobalTime = DISABLE;
	return HAL_CAN_AddTxMessage(&hcan, &txh, emptydata, &can_tx_mailbox_used);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *pcan) {
	CAN_RxHeaderTypeDef rxh = {0};
	uint8_t data[8] = {0};
	if (HAL_CAN_GetRxMessage(pcan, CAN_RX_FIFO0, &rxh, data) == HAL_OK) {
		uint8_t from = (uint8_t)rxh.ExtId;
		if(cmdlis) {
			cmdlis(data, rxh.DLC, from);
		}
	}
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *pcan) {
	CAN_RxHeaderTypeDef rxh = {0};
	uint8_t data[8] = {0};
	if (HAL_CAN_GetRxMessage(pcan, CAN_RX_FIFO1, &rxh, data) == HAL_OK) {
		uint8_t from = (uint8_t)rxh.ExtId;
		_Bool isok = ((rxh.ExtId >> 18) & REPLY_OK_HEADER) == REPLY_OK_HEADER;
		if(replis) {
			replis(isok, from);
		}
	}
}
