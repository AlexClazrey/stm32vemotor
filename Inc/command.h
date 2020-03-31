#ifndef __COMMAND_H__
#define __COMMAND_H__
#include "stm32f1xx.h"
#include "lm.h"
#include "config.h"

#if WIFI_ENABLE==1
#include "wifi8266/wifi_8266_mod.h"

Wifi_HandleTypeDef *wifi_gethandler();
void wifi_rx_to_uart();
void wifi_autosetup_tasklist();
void wifi_greet_1();
void wifi_parse_cmd(struct lm_handle* plmh);
void wifi_tick_callback(Wifi_HandleTypeDef* phwifi, WifiRtnState state, int index, int finished);
#endif

void command_read(struct lm_handle* plmh);
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
	input_wifi, // 输入了WiFi命令时候的信号。
};

#endif
