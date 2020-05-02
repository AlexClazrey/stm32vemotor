#ifndef __COMMAND_H__
#define __COMMAND_H__
#include "stm32f1xx.h"
#include "lm.h"
#include "config.h"
#include "led.h"

#if WIFI_ENABLE==1
#include "wifi8266/wifi_8266_mod.h"

Wifi_HandleTypeDef *wifi_gethandler();
void wifi_rx_to_uart();
void wifi_autosetup_tasklist();
void wifi_greet_1();
void wifi_parse_cmd(struct lm_handle* plmh);
void wifi_tick_callback(Wifi_HandleTypeDef* phwifi, WifiRtnState state, int index, int finished);
void wifi_send_tasklist(const char *str, int normalmode);
#else
void wifi_send_tasklist(const char *str, int normalmode);
#endif

void command_read(struct lm_handle* plmh);
void cmd_can_isr(char *data, size_t len, uint16_t from, _Bool isbroadcast, struct lm_handle *plmhandle);

// 在主循环里面调用这个，当 motor cycle 打开的时候这个函数负责发布命令
void mcycle_cmd(struct lm_handle *plmh, uint32_t count);

enum cmdtype {
	cmd_empty = 0,
	cmd_motor_stop,
	cmd_motor_speed,
	cmd_motor_reset,
	cmd_motor_set_home,
	cmd_motor_pos,
	cmd_motor_relapos,
	cmd_motor_percent,
	cmd_motor_where,
	cmd_setting_id,
	cmd_setting_mcycle,
	cmd_setting_lm_limit_in,
	cmd_setting_lm_limit_out,
	cmd_setting_load,
	cmd_setting_show_id,
	cmd_wifi_check,
	cmd_wifi_auto,
	cmd_wifi_join,
	cmd_wifi_leave,
	cmd_wifi_tcp_connect,
	cmd_wifi_tcp_drop,
	cmd_wifi_setap,
	cmd_wifi_settcp,
	cmd_wifi_dummy,
	cmd_led_color,
	cmd_led_color_grad_to,
};

enum cmdfrom {
	CMD_FROM_NONE,
	CMD_FROM_SERIAL,
	CMD_FROM_WIFI,
};

struct cmd {
	enum cmdtype type;
	enum cmdfrom from;
	union {
		struct {
			union {
				int32_t pos;
				int32_t relapos;
				int32_t speed;
			};
			union {
				uint8_t hardstop;
				uint8_t dir;
				uint8_t percent;
			};
		} motorcmd;
		struct{ 
			struct rgb color; 
			uint16_t ms;
		} ledcmd;
		struct {
			union {
				uint16_t machine_id;
				int32_t lm_limit_in;
				int32_t lm_limit_out;
			};
		} settingcmd;
	};
	uint16_t receiver;
	uint16_t sender;
	
};

enum inputcmdtype {
	input_error, // 输入出错时候的信号
	input_empty, // 没有输入到行尾的时候的返回信号
	input_next,  // 连续两个行尾，比如说输入了一个空行的时候的信号
	input_lmcmd, // 输入了一个本机的电机命令时候的信号
	input_settings, // 输入了一个本机设置命令时候的信号
	input_led, // 输入 led 命令时候的信号
	input_can, // 输入了CAN命令时候的信号。
	input_wifi, // 输入了WiFi命令时候的信号。
};

#endif
