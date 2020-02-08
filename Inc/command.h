#ifndef __COMMAND_H__
#define __COMMAND_H__
#include "stm32f1xx.h"
#include "lm.h"

void inputbuf_read(struct lm_handle *plmh);
char* inputbuf_get();
void inputbuf_setend(uint32_t end);

HAL_StatusTypeDef can_cmd_send(struct lm_cmd *cmd, uint8_t receiver_id);
int canbuf_read(struct lm_cmd *dest, char *data, size_t len);
void mcycle_cmd(struct lm_handle *plmh, uint32_t count);

enum inputcmdtype {
	input_error, // 输入出错时候的信号
	input_empty, // 没有输入到行尾的时候的返回信号
	input_next,  // 连续两个行尾，比如说输入了一个空行的时候的信号
	input_lmcmd, // 输入了一个本机的电机命令时候的信号
	input_settings, // 输入了一个本机设置命令时候的信号
	input_can, // 输入了CAN命令时候的信号。
};

#endif
