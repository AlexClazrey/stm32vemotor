#ifndef __CAN_IO_H_
#define __CAN_IO_H_
#include "stm32f1xx_hal.h"

typedef void(*CAN_MsgListener)(uint8_t* data, uint8_t len);
#define CAN_ADDRESS 0x244
#define CAN_LISTEN_TO 0x245

extern CAN_HandleTypeDef hcan;
extern CAN_RxHeaderTypeDef can_rx_header;
extern CAN_TxHeaderTypeDef can_tx_header;
extern uint32_t can_tx_mailbox;

void can_init();
void can_listener(CAN_MsgListener listener);
HAL_StatusTypeDef can_msg_add(uint8_t* data, uint8_t len);
void can_msg_get();

#endif
