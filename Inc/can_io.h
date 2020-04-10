#ifndef __CAN_IO_H_
#define __CAN_IO_H_
#include "stm32f1xx_hal.h"


extern uint32_t can_tx_mailbox_used;

typedef void(*CAN_CmdListener)(uint8_t* data, uint8_t len, uint8_t from);
typedef void(*CAN_ReplyListener)(_Bool ok, uint8_t from);

void can_init(uint8_t selfid);
void can_set_cmdlistener(CAN_CmdListener lis);
void can_set_replylistener(CAN_ReplyListener lis);
void can_set_id(uint8_t id);
HAL_StatusTypeDef can_send_cmd(uint8_t *data, uint8_t len, uint8_t to);
HAL_StatusTypeDef can_send_reply(uint8_t to, _Bool ok);


#endif
