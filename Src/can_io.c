#include "can_io.h"

CAN_RxHeaderTypeDef can_rx_header = { 0 };
CAN_TxHeaderTypeDef can_tx_header = { 0 };
CAN_FilterTypeDef can_filter = { 0 };
uint32_t can_tx_mailbox = 0;
static CAN_MsgListener can_lis = NULL;

void can_init() {
	can_tx_header.DLC = 8;
	can_tx_header.IDE = CAN_ID_STD;
	can_tx_header.RTR = CAN_RTR_DATA; // Data Frame or Remote Request Frame;
	can_tx_header.StdId = CAN_ADDRESS;

	can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	can_filter.FilterIdHigh = CAN_LISTEN_TO << 5;
	can_filter.FilterIdLow = 0;
	can_filter.FilterMaskIdHigh = 0;
	can_filter.FilterMaskIdLow = 0;
	can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
	can_filter.FilterActivation = CAN_FILTER_ENABLE;
	HAL_CAN_ConfigFilter(&hcan, &can_filter);

	HAL_CAN_Start(&hcan);
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void can_set_listener(CAN_MsgListener listener) {
	can_lis = listener;
}

// data can have 8 bytes at most.
HAL_StatusTypeDef can_msg_add(uint8_t* data, uint8_t len) {
	if (len > 8) {
		return HAL_ERROR;
	}
	can_tx_header.DLC = len;
	// 这里的函数直接会读入 header 的数据操作不保存引用，所以可以安全复用不担心 header 数据出错。
	return HAL_CAN_AddTxMessage(&hcan, &can_tx_header, data, &can_tx_mailbox);
}

static uint8_t canbuf[10] = { 0 };
void can_msg_get() {
	// I set interrupt for FIFO0.
	if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &can_rx_header, canbuf)
			== HAL_OK) {
		if (can_lis) {
			can_lis(canbuf, can_rx_header.DLC);
		}
	}
}
