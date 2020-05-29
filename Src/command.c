#include "command.h"
#include "DSpin/dspin.h"
#include "util.h"
#include "cycletick.h"
#include "log_uart.h"
#include "inputbuf.h"

#include "can_io.h"
#include "led.h"
#include "flash.h"
#include "main.h"

#if WIFI_ENABLE == 1
#include "wifi8266/wifi_8266_mod.h"
#endif

#include <string.h>
#include <stdio.h>

/* Configuration 设置项目 */
#include "config.h"

#if WIFI_ENABLE == 1
// WiFi
// 这里写得丑了一点强行引用
extern UART_HandleTypeDef huart2;
static Wifi_HandleTypeDef hwifi = { .huart = &huart2 };
static int wifi_change_to_raw_when_success = 0;
static int wifi_pipe_raw = 0;
static _Bool wifi_reconnecting = 0;
#endif

// motor cycle test
static int lm_cycle = 0; // 循环测试开启指示
static uint32_t lm_cycle_pause_count = 0; // 暂停的等待计数
static uint32_t lm_cycle_speed_count = 0; // 在过热无反应时候的等待计数

/* Functions */
// ------------- Main Function
void uart_user_inputbuf_read(struct lm_handle *plmh);
void command_switch_read(struct lm_handle *plmhandle);
void command_read(struct lm_handle *plmhandle) {
	uart_user_inputbuf_read(plmhandle);
	command_switch_read(plmhandle);
	if(wifi_reconnecting && cycletick_everyms(5000 + machine_id * 5)) {
		logu_s(LOGU_INFO, "Wifi reconnect started.");
		wifi_autosetup_tasklist();
		wifi_reconnecting = 0;
	}
}
// ------------- Switch Process
int sw2_previous = 0, sw3_previous = 0;
int sw23_count = 0;
static struct rgb hue_to_rgb(uint16_t hue, uint8_t sat, uint8_t val);
void command_switch_read(struct lm_handle *plmhandle) {
	if(sw2_pressed() && sw3_pressed()) {
		if(!sw2_previous || !sw3_previous)
			lm_append_newcmd(plmhandle, lm_cmd_stop, 0, 0);
		else
			sw23_count++;
		// treat sw23 as hue
		struct rgb color = hue_to_rgb(sw23_count % 360, 100, 100);
		led_set(&color);
	} else if (sw2_pressed()) {
		if(!sw2_previous)
			lm_append_newcmd(plmhandle, lm_cmd_speed, 20000, 0);
	} else if (sw3_pressed()) {
		if(!sw3_previous)
			lm_append_newcmd(plmhandle, lm_cmd_speed, 20000, 1);
	} else {
		if(sw2_previous || sw3_previous)
			lm_append_newcmd(plmhandle, lm_cmd_stop, 0, 0);
		sw23_count = 0;
	}
	sw2_previous = sw2_pressed();
	sw3_previous = sw3_pressed();
}

// sat is 0-100, val is 0-100, hue is 0-360
static struct rgb hue_to_rgb(uint16_t hue, uint8_t sat, uint8_t val) {
	uint16_t c = sat * val / 100;
	uint16_t x = c * (60 - ABS((int)hue % 120 - 60)) / 60;
	uint16_t m = val - c;
	uint32_t r = 0, g = 0, b = 0;
	if(hue < 60) {
		r = c;
		g = x;
	} else if (hue < 120) {
		r = x;
		g = c;
	} else if (hue < 180) {
		g = c;
		b = x;
	} else if (hue < 240) {
		g = x;
		b = c;
	} else if (hue < 300) {
		r = x;
		b = c;
	} else {
		r = c;
		b = x;
	}
	struct rgb res;
	res.r = (r+m)*255/100;
	res.g = (g+m)*255/100;
	res.b = (b+m)*255/100;
	return res;
}

// ------------- Input Buffer Process
static void input_feedback(enum cmdfrom from, int success);
static enum inputcmdtype cmd_read_act(const char *src, struct lm_handle *plmh, enum cmdfrom from, int suppress_error);
static enum inputcmdtype cmd_parse(const char *cmd, struct cmd *out_store);

void uart_user_inputbuf_read(struct lm_handle *plmhandle) {
	// 这个函数把解析InputBuffer的工作从中断里面遇到行尾设置结束符，
	// 改成移动到主循环里面，这样可以一个周期里面对超过一条的命令汇入。
	// 首先先向后寻找\r\n或者\0的位置，复制到cmdbuf里面，然后清理cmdbuf，然后解析。
	// 为了能在一次主循环里面处理多条命令。
	// 需要创建不止一个命令缓存而是很多个命令缓存池
	// 如果命令缓存池还有一个空位那就尝试读取
	// 因为不是自己的命令会沿着CAN发送出去不会占用空位
	// 如果是自己的命令那么占用一个位置
	// 如过缓存池被用满了那么停止这里的Read Input同时应该发出一个警告。
	// 发送很多个CAN指令也可能消耗很多时间，所以这里还需要在Parse一个新的命令之前注意检查时间，
	// 如果时间不多了那么就留到下一个循环处理。
	_Bool lineend = 0;
	const char* newinput = inputbuf_read_toline(getuserbuf(), &lineend);
#if WIFI_ENABLE == 1
	if (wifi_pipe_raw) {
		// 如果开启WIFI透传，那么把用户在串口上的输入直接写入WIFI
		// 同时给出 Echo
		// 按下 Ctrl+D (ASCII EOT) 退出透传模式
		if(newinput != NULL) {
			if (newinput[0] == 0x04) {
				wifi_pipe_raw = 0;
				wifi_task_add(&hwifi, wifi_stopsend_unvarnished);
				logu_s(LOGU_WARN, "Leave wifi pipe mode.");
			} else {
				uint32_t len = strlen(newinput);
				wifi_send_raw(&hwifi, newinput, len);
				logu_raw(newinput, len);
			}
		}
	} else {
#else
	newinput = newinput; // get rid of a GCC unused warning
#endif
		while (cycletick_now() < 8 && lineend) {
			// now it gets a new line
			if (!lm_hasspace(plmhandle)) {
				logu_s(LOGU_WARN, "Stop reading input due to full lm_cmd queue");
				return;
			}
			enum inputcmdtype type = cmd_read_act(inputbuf_getline(getuserbuf()), plmhandle, CMD_FROM_SERIAL, 0);
			if (type == input_empty)
				break;
			inputbuf_read_toline(getuserbuf(), &lineend);
		}
#if WIFI_ENABLE == 1
	}
#endif
}

HAL_StatusTypeDef cmd_can_send(struct cmd *cmd);
static HAL_StatusTypeDef cmd_cansendfeedback(enum cmdfrom from, HAL_StatusTypeDef send_res);
static _Bool cmd_action(struct cmd* cmd, struct lm_handle* plmh);
static enum inputcmdtype cmd_read_act(const char *src, struct lm_handle *plmh, enum cmdfrom from, int suppress_error) {
	struct cmd cmd = { 0 };
	cmd.from = from;
	// any input will turn off motor cycle test
	if (lm_cycle) {
		logu_f(LOGU_INFO, "Cycle off.");
		lm_cycle = 0;
	}
	enum inputcmdtype type = cmd_parse(src, &cmd);
	if (type == input_error) {
		// Error details was printed in parse function
		if (!suppress_error) {
			input_feedback(from, false);
		}
	} else if (type == input_next) {
		// do nothing here
		// 在 parse 的时候会输出一句 debug 表示又读到一句空行
	} else if (type == input_empty) {
		// 读到empty表示输入缓冲区已经读完了。
	} else if (type == input_can) {
		HAL_StatusTypeDef hs = cmd_can_send(&cmd);
		cmd_cansendfeedback(from, hs);
		HAL_Delay(1); // TODO 这里强行停顿不太好，但是CAN SEND需要一点时间。
		input_feedback(from, hs == HAL_OK);
	} else {
		input_feedback(from, cmd_action(&cmd, plmh));
	}
	return type;
}

static void cmd2lmcmd(struct cmd* cmd, struct lm_cmd* lmcmd) {
	switch(cmd->type) {
		case cmd_motor_stop:
		lmcmd->type = lm_cmd_stop;
		lmcmd->dir_hard = cmd->motorcmd.dir;
		break;
		case cmd_motor_speed:
		lmcmd->type = lm_cmd_speed;
		lmcmd->pos_speed = cmd->motorcmd.speed;
		lmcmd->dir_hard = cmd->motorcmd.dir;
		break;
		case cmd_motor_reset:
		lmcmd->type = lm_cmd_reset;
		break;
		case cmd_motor_set_home:
		lmcmd->type = lm_cmd_set_home;
		break;
		case cmd_motor_pos:
		lmcmd->type = lm_cmd_pos;
		lmcmd->pos_speed = cmd->motorcmd.pos;
		break;
		case cmd_motor_relapos:
		lmcmd->type = lm_cmd_relapos;
		lmcmd->pos_speed = cmd->motorcmd.pos;
		break;
		case cmd_motor_percent:
		lmcmd->type = lm_cmd_pos;
		lmcmd->pos_speed = (int) ((double) cmd->motorcmd.percent * (lm_conf_limit_out - lm_conf_limit_in) / 100 + lm_conf_limit_in);
		break;
		case cmd_motor_where:
		lmcmd->type = lm_cmd_where;
		break;
		default:
		logu_f(LOGU_ERROR, "unknown cmd convert: %d.", (int)cmd->type);
	}
}

static char idstr[12] = {0}; // WiFi Send needs a global memory
static void input_feedback_raw(enum cmdfrom from, char* cstr);
// return true for success
static _Bool cmd_action(struct cmd* cmd, struct lm_handle* plmh) {
	struct lm_cmd lmcmd = {0};
	int32_t target;
	// any input except "mcycle" will turn off motor cycle test
	if (lm_cycle) {
		logu_f(LOGU_INFO, "Cycle off.");
		lm_cycle = 0;
	}
	switch(cmd->type) {
		case cmd_motor_stop:
		case cmd_motor_speed:
		case cmd_motor_reset:
		case cmd_motor_set_home:
		case cmd_motor_pos:
		case cmd_motor_relapos:
		case cmd_motor_percent:
		case cmd_motor_where:
		cmd2lmcmd(cmd, &lmcmd);
		lm_append_cmd(plmh, &lmcmd);
		break;
		case cmd_setting_id:
		if(cmd->settingcmd.machine_id == 0) {
			logu_s(LOGU_ERROR, "Machine id can't be 0.");
			return 0;
		}
		if(flash_save_machineid(cmd->settingcmd.machine_id) != HAL_OK) {
			return 0;
		}
		machine_id = cmd->settingcmd.machine_id;
		can_set_id(machine_id);
		break;
		case cmd_setting_show_id:
		snprintf(idstr, 12, "<%hu>\r\n", machine_id);
		input_feedback_raw(cmd->from, idstr);
		break;
		case cmd_setting_mcycle:
		lm_cycle = !lm_cycle;
		if (lm_cycle) {
			// cycle kick start action, move to cycle_in position
			lmcmd.type = lm_cmd_pos;
			lmcmd.pos_speed = lm_cycle_in;
			lm_append_cmd(plmh, &lmcmd);
		}
		break;
		case cmd_setting_lm_limit_in:
		if(cmd->settingcmd.lm_limit_in != UINT32_MAX) {
			target = cmd->settingcmd.lm_limit_in;
		} else {
			target = lm_read_pos();
		}
		if(flash_save_motor_limit_in(target) != HAL_OK) {
			return 0;
		}
		lm_conf_limit_in = target;
		break;
		case cmd_setting_lm_limit_out:
		if(cmd->settingcmd.lm_limit_out != UINT32_MAX) {
			target = cmd->settingcmd.lm_limit_out;
		} else {
			target = lm_read_pos();
		}
		if(flash_save_motor_limit_out(target) != HAL_OK) {
			return 0;
		}
		lm_conf_limit_out = target;
		break;
		case cmd_setting_load:
		load_configurations();
		break;
		case cmd_wifi_check:
		case cmd_wifi_auto:
		case cmd_wifi_join:
		case cmd_wifi_leave:
		case cmd_wifi_tcp_connect:
		case cmd_wifi_tcp_drop:
		case cmd_wifi_setap:
		case cmd_wifi_settcp:
		case cmd_wifi_dummy:
		// TODO wifi command 在 parse 的时候直接执行了，这里永远返回 true
		break;
		case cmd_led_color:
		led_set(&(cmd->ledcmd.color));
		break;
		case cmd_led_color_grad_to:
		led_gradient_to(&(cmd->ledcmd.color), cmd->ledcmd.ms);
		break;
		default:
		logu_f(LOGU_ERROR, "Action unknown cmd: %d.", (int)cmd->type);
	}
	return 1;
}

static void input_feedback_raw(enum cmdfrom from, char* cstr) {
	if (from == CMD_FROM_SERIAL) {
		logu_raw(cstr, strlen(cstr));
	} else if (from == CMD_FROM_WIFI) {
		wifi_send_tasklist(cstr, 0);
	} else {
		logu_s(LOGU_WARN, "Can't feedback to CAN");
		logu_raw(cstr, strlen(cstr));
	}
}
static void input_feedback(enum cmdfrom from, int success) {
	if (success) {
		input_feedback_raw(from, "<OK>\r\n");
	} else {
		input_feedback_raw(from, "<FAIL>\r\n");
	}
}

static HAL_StatusTypeDef cmd_cansendfeedback(enum cmdfrom from, HAL_StatusTypeDef send_res) {
	if (from == CMD_FROM_SERIAL) {
		if (send_res == HAL_OK) {
			logu_f(LOGU_DEBUG, "CAN Send OK.");
		} else {
			logu_f(LOGU_ERROR, "CAN Send failed.");
		}
	} else if (from == CMD_FROM_WIFI) {
		if (send_res == HAL_OK) {
			wifi_send_tasklist("CAN Send OK.\r\n", 0);
		} else {
			wifi_send_tasklist("CAN Send Failed.\r\n", 0);
		}
	}
	return send_res;
}

// --------- Command

/* Command format is:
 * [#<id>](mr|ms|mp|mpp|mreset|mhome|mcycle|mid)[ <args>]
 */

static int cmd_parse_body(const char *cmd, struct cmd *store);
static int read_mr_args(const char *cmd, uint32_t *out_speed, uint32_t *out_dir);
static enum inputcmdtype cmd_parse(const char *cmd, struct cmd *out_store) {
	uint16_t receiver = (uint16_t) -1;

	logu_f(LOGU_TRACE, "Parse: %s", cmd);

	// 为了兼容之前的格式
	if (strncmp(cmd, "sig ", 4) == 0) {
		cmd += 4;
	}

	// if command has an receiver
	if (cmd[0] == '#') {
		size_t scanlen;
		if (sscanf(++cmd, "%hu%n", &receiver, &scanlen) != 1 || receiver > 65535) {
			logu_s(LOGU_ERROR, "Cmd id read error.");
			return 1;
		}
		out_store->receiver = receiver;
		out_store->sender = machine_id;
		if (cmd[scanlen] == ' ') {
			// 兼容ID后面有或者没有空格两种情况
			cmd++;
		}
		cmd += scanlen;
	}

	// parse command body
	int res = cmd_parse_body(cmd, out_store);

	if (res == 1) {
		return input_error;
	}
	if (receiver == machine_id || receiver == (uint16_t) -1) {
		if (res == 0) {
			return input_lmcmd;
		} else if (res == 2) {
			return input_settings;
		} else if (res == 3) {
			return input_wifi;
		} else if (res == 4) {
			return input_led;
		} else {
			logu_f(LOGU_ERROR, "Unknown error [02]");
			return input_error;
		}
	} else {
		return input_can;
	}
}

#if WIFI_ENABLE==1
static char wifiscanssid[64];
static char wifiscanpwd[64];
static char wifiscanip[64];
#endif

// return 0: lm_cmd command, command which actually modifies lm_cmd* store.
// return 1: parse failed
// return 2: settings command
// return 3: wifi command
// return 4: led command
static int cmd_parse_body(const char *cmd, struct cmd *store) {
	if (cmd[0] == 'm') {
		if (strncmp(cmd + 1, "cycle", 6) == 0) {
			store->type = cmd_setting_mcycle;
			return 2;
		} else if (strncmp(cmd + 1, "home", 5) == 0) {
			store->type = cmd_motor_set_home;
		} else if (strncmp(cmd + 1, "reset", 6) == 0) {
			store->type = cmd_motor_reset;
		} else if (strncmp(cmd + 1, "id ", 3) == 0) {
			uint16_t id;
			store->type = cmd_setting_id;
			if (sscanf(cmd + 4, "%hu", &id) == 1 && id < 65536 && id > 0) {
				store->settingcmd.machine_id = id;
				return 2;
			} else {
				logu_s(LOGU_ERROR, "Read Machine Id Error, should between 1 and 65535.");
				return 1;
			}
		} else if (cmd[1] == 'r' && cmd[2] == ' ') {
			// 这里一定要注意检验 cmd[2] 是空格，不然可能是NUL字符，就是一定要连续检验，想不到还有这个隐患。
			uint32_t pos, dir;
			if (read_mr_args(cmd + 3, &pos, &dir) != 2) {
				logu_s(LOGU_ERROR, "Read mr command failed.");
				return 1;
			}
			store->type = cmd_motor_speed;
			store->motorcmd.pos = pos;
			store->motorcmd.dir = dir;
		} else if (cmd[1] == 's' && cmd[2] == '\0') {
			store->type = cmd_motor_stop;
		} else if (cmd[1] == 'p') {
			if (cmd[2] == ' ') {
				store->type = cmd_motor_pos;
				if (sscanf(cmd + 3, "%ld", &(store->motorcmd.pos)) != 1) {
					logu_s(LOGU_ERROR, "Read mp command failed.");
					return 1;
				}
			} else if (cmd[2] == 'p' && cmd[3] == ' ') {
				store->type = cmd_motor_percent;
				uint16_t percent;
				if (sscanf(cmd + 4, "%hu", &percent) != 1) {
					logu_s(LOGU_ERROR, "Read mpp percent failed.");
					return 1;
				}
				if (percent > 100) {
					logu_s(LOGU_ERROR, "Mpp percent should between 0 and 100.");
					return 1;
				}
				store->motorcmd.percent = percent;
			} else if (cmd[2] == 'r' && cmd[3] == ' ') {
				store->type = cmd_motor_relapos;
				if (sscanf(cmd + 4, "%ld", &(store->motorcmd.pos)) != 1) {
					logu_s(LOGU_ERROR, "Read mp command failed.");
					return 1;
				}
			}
		} else if (strncmp(cmd + 1, "setout", 7) == 0) {
			store->type = cmd_setting_lm_limit_out;
			store->settingcmd.lm_limit_out = UINT32_MAX;
		} else if (strncmp(cmd + 1, "setin", 6) == 0) {
			store->type = cmd_setting_lm_limit_in;
			store->settingcmd.lm_limit_in = UINT32_MAX;
		} else if (strncmp(cmd + 1, "load", 5) == 0) {
			store->type = cmd_setting_load;
		} else if (strncmp(cmd + 1, "where", 6) == 0) {
			store->type = cmd_motor_where;
		} else {
			logu_s(LOGU_ERROR, "Command parse failed");
			return 1;
		}
	} else if (cmd[0] == 'w') {
#if WIFI_ENABLE==1
		// TODO 在 wifi 的 task 没有完全释放的时候不要执行下一个命令
		// 应该向上Log wifi busy这句话
		if (strncmp(cmd + 1, "at", 3) == 0) {
			store->type = cmd_wifi_check;
			wifi_task_add(&hwifi, wifi_checkat);
		} else if (strncmp(cmd + 1, "auto", 5) == 0) {
			// set up wifi
			store->type = cmd_wifi_auto;
			wifi_autosetup_tasklist();
		} else if (strncmp(cmd + 1, "join", 4) == 0) {
			store->type = cmd_wifi_join;
			// join ap
			if (cmd[1 + 4] == '\0') {
				wifi_task_add_withargs(&hwifi, wifi_joinap_args, NULL, 2, (int[] ) { (int) wifi_conf_ssid, (int) wifi_conf_pwd });
			} else {
				if (strlen(cmd) > 64) {
					logu_s(LOGU_ERROR, "SSID and pwd are too long.");
					return 1;
				}
				int scnt = sscanf(cmd + 5, "%s %s", wifiscanssid, wifiscanpwd);
				if (scnt != 2) {
					logu_s(LOGU_ERROR, "SSID or pwd parsed failed.");
					return 1;
				}
				wifi_task_add_withargs(&hwifi, wifi_joinap_args, NULL, 2, (int[] ) { (int) wifiscanssid,
								(int) wifiscanpwd });
			}
		} else if (strncmp(cmd + 1, "tcp", 3) == 0) {
			store->type = cmd_wifi_tcp_connect;
			// tcp connect
			if (cmd[1 + 3] == '\0') {
				wifi_task_add_withargs(&hwifi, wifi_tcpconn_args, NULL, 2, (int[] ) { (int) wifi_conf_tcpip,
								(int) wifi_conf_tcpport });
			} else {
				if (strlen(cmd) > 64) {
					logu_s(LOGU_ERROR, "IP and port are too long.");
					return 1;
				}
				uint16_t wifiscanport;
				int scnt = sscanf(cmd + 4, "%s %hu", wifiscanip, &wifiscanport);
				if (scnt != 2) {
					logu_s(LOGU_ERROR, "IP or port parsed failed.");
					return 1;
				}
				wifi_task_add_withargs(&hwifi, wifi_tcpconn_args, NULL, 2, (int[] ) { (int) wifiscanip,
								(int) wifiscanport });
			}
		} else if (strncmp(cmd + 1, "send", 5) == 0) {
			// start send and set serial input into raw mode
			store->type = cmd_wifi_dummy;
			wifi_task_add(&hwifi, wifi_startsend_unvarnished);
			wifi_change_to_raw_when_success = 1;
			logu_s(LOGU_WARN, "Start wifi pipe mode.");
		} else if (strncmp(cmd + 1, "drop", 5) == 0) {
			store->type = cmd_wifi_tcp_drop;
			// tcp disconnect
			wifi_task_add(&hwifi, wifi_dropsingleconn);
		} else if (strncmp(cmd + 1, "leave", 6) == 0) {
			store->type = cmd_wifi_leave;
			// leave ap
			wifi_task_add(&hwifi, wifi_leaveap);
		} else if (strncmp(cmd + 1, "setap ", 6) == 0) {
			store->type = cmd_wifi_setap;
			char tmpssid[WIFI_STRSIZE], tmppwd[WIFI_STRSIZE];
			if(sscanf(cmd + 7, "%s %s", tmpssid, tmppwd) != 2) {
				logu_s(LOGU_ERROR, "Wifi AP input read error");
				return 1;
			}
			// action part
			if(flash_save_wifissid(tmpssid) != HAL_OK) {
				logu_s(LOGU_ERROR, "Flash save wifi ssid error.");
				return 1;
			}
			if(flash_save_wifipwd(tmppwd) != HAL_OK) {
				logu_s(LOGU_ERROR, "Flash save wifi pwd error.");
				return 1;
			}
			strncpy(wifi_conf_ssid, tmpssid, WIFI_STRSIZE);
			strncpy(wifi_conf_pwd, tmppwd, WIFI_STRSIZE);
			return 2;
		} else if (strncmp(cmd + 1, "settcp ", 7) == 0) {
			store->type = cmd_wifi_settcp;
			char tmpip[WIFI_STRSIZE];
			uint16_t tmpport;
			if(sscanf(cmd + 8, "%s %hu", tmpip, &tmpport) != 2) {
				logu_s(LOGU_ERROR, "Wifi tcp ip port input read error");
				return 1;
			}
			// action part
			if(flash_save_tcpip(tmpip) != HAL_OK) {
				logu_s(LOGU_ERROR, "Flash save tcp ip error.");
				return 1;
			}
			if(flash_save_tcpport(tmpport) != HAL_OK) {
				logu_s(LOGU_ERROR, "Flash save tcp port error.");
				return 1;
			}
			strncpy(wifi_conf_tcpip, tmpip, WIFI_STRSIZE);
			wifi_conf_tcpport = tmpport;
			return 2;
		} else {
			logu_s(LOGU_ERROR, "Unknown wifi commmand.");
			return 1;
		}
		return 3;
#else
		logu_s(LOGU_ERROR, "Command parse failed, Wifi module is disabled.");
		return 1;
#endif

	} else if (strncmp(cmd, "led ", 4) == 0) {
		uint16_t r, g, b;
		int scnt = sscanf(cmd + 4, "%hu%hu%hu", &r, &g, &b);
		if(scnt != 3) {
			logu_s(LOGU_ERROR, "Led brightness parse failed.");
			return 1;
		}
		if(r > 255 || g > 255 || b > 255) {
			logu_s(LOGU_ERROR, "Led brightness too high.");
			return 1;
		}
		store->type = cmd_led_color;
		store->ledcmd.color.r = r;
		store->ledcmd.color.g = g;
		store->ledcmd.color.b = b;
		return 4;
	} else if (strncmp(cmd, "ledgrad ", 8) == 0) {
		uint16_t r, g, b, intv;
		int scnt = sscanf(cmd + 8, "%hu%hu%hu%hu", &r, &g, &b, &intv);
		if(scnt != 4) {
			logu_s(LOGU_ERROR, "Led gradient parse failed.");
			return 1;
		}
		if(r > 255 || g > 255 || b > 255) {
			logu_s(LOGU_ERROR, "led brightness too high.");
			return 1;
		}
		store->type = cmd_led_color_grad_to;
		store->ledcmd.color.r = r;
		store->ledcmd.color.g = g;
		store->ledcmd.color.b = b;
		store->ledcmd.ms = intv;
		return 4;
	} else if (cmd[0] == 's') {
		if(strncmp(cmd + 1, "id", 2) == 0) {
			store->type = cmd_setting_show_id;
			return 2;
		} else {
			return 1;
		}
	} else {
		logu_s(LOGU_ERROR, "Command parse failed");
		return 1;
	}
	return 0;
}

static int read_mr_args(const char *cmd, uint32_t *out_speed, uint32_t *out_dir) {
	uint32_t dir, speed;
	// speed 1 ~ 30
	// 太高的速度在冲击时会引发L6470过流关闭，需要 L6470 Reset 指令重新开启
	uint32_t minsp = 1, maxsp = 30;
	if (sscanf(cmd, "%lu %lu", &dir, &speed) != 2) {
		logu_s(LOGU_ERROR, "Parse args failed.");
		return 0;
	}
	if (speed < minsp || speed > maxsp) {
		logu_f(LOGU_ERROR, "Speed not in range: %d, range is %d ~ %d.", speed, minsp, maxsp);
		return 0;
	}
	if (dir != 0 && dir != 1) {
		logu_s(LOGU_ERROR, "Dir should be 0 or 1.");
		return 0;
	}
	*out_speed = speed * 1000;
	*out_dir = dir;
	return 2;
}

#if WIFI_ENABLE ==1 

// WiFi receive
// 先用一个缓冲区收集输入，然后在主循环里面调取解析函数
// 解析函数可以使用 inputbuf_read_one_cmd

static char wifi_command_buffer[WIFI_RECV_BUFFER_SIZE] = {0};
void wifi_parse_cmd(struct lm_handle *plmh) {
	const char *wifi_raw = wifi_rx_cap(&hwifi);
	if (wifi_raw == NULL)
		return;
	const char *ipdstr = strstr(wifi_raw, "\r\n+IPD,");
	if (ipdstr != NULL) {
		const char *s1 = strchr(ipdstr, ':');
		if (s1 == NULL) {
			logu_f(LOGU_ERROR, "WiFi received an invalid data: %s", wifi_raw);
			return;
		}
		wifi_raw = s1 + 1;
		memset(wifi_command_buffer, 0, WIFI_RECV_BUFFER_SIZE);
		strcpy(wifi_command_buffer, wifi_raw);
		logu_f(LOGU_INFO, "WiFi received: %s", wifi_command_buffer);
		char *cmd = wifi_command_buffer;
		int cmdlen;
		while((cmdlen = strcspn(cmd, "\r\n")) > 0) {
			cmd[cmdlen] = 0;
			cmd_read_act(cmd, plmh, CMD_FROM_WIFI, 0);
			cmd += cmdlen + 1;
			if(*cmd == '\n') {
				// in case of \r\n
				cmd++;
			}
		}
	}
}

Wifi_HandleTypeDef* wifi_gethandler() {
	return &hwifi;
}

void wifi_rx_to_uart() {
	const char *wrx = wifi_rx_cap(&hwifi);
	size_t wrxlen = wifi_rx_cap_len(&hwifi);
	if (wrx) {
		if (wifi_pipe_raw) {
			logu_raw(wrx, wrxlen);
		} else {
			logu_s(LOGU_INFO, "wifi prints:");
			logu_raw(wrx, wrxlen);
		}
	}
}

static char wifi_autosetup_greet_buf[50];
static const char
		*wifi_autosetup_tasklist_name = "Auto Setup",
		*wifi_autosetup_joinap_taskname = "join ap",
		*wifi_autosetup_tcpconn_taskname = "tcp conn",
		*wifi_autosetup_at_check = "at check";
static const char *wifi_startsend_taskname = "start send";
void wifi_autosetup_tasklist() {
	// 1. Exit Send Mode
	// 2. check "AT" (to remove noise input)
	// 3. AT+CWMODE=1
	// 4. AT+CIPMUX=0
	// 5. AT+CIPMODE=1
	// 6. SET DHCP ON
	// 7. JOIN AP
	// 8. CONNECT TCP
	// 9. Start send mode
	// 10. Send Hello
	// 11. delay
	// 12. Exit send mode
	// 13. check "AT"
	logu_s(LOGU_INFO, "Start WiFi Auto Setup task list.");
	snprintf(wifi_autosetup_greet_buf, 50, "Machine %hu greetings ($ a $).\r\n", machine_id);
	wifi_task_setlistname(&hwifi, wifi_autosetup_tasklist_name);
	wifi_task_add(&hwifi, wifi_stopsend_unvarnished);
	wifi_task_add(&hwifi, wifi_checkat);
	wifi_task_add_withname(&hwifi, wifi_checkat, wifi_autosetup_at_check);
	wifi_task_add(&hwifi, wifi_setmodewifi_client);
	wifi_task_add(&hwifi, wifi_setsingleconn);
	wifi_task_add(&hwifi, wifi_setmodetrans_normal);
	wifi_task_add(&hwifi, wifi_setdhcp_on);
	wifi_task_add_withargs(&hwifi, wifi_joinap_args, wifi_autosetup_joinap_taskname, 2, (int[] ) { (int) wifi_conf_ssid,
					(int) wifi_conf_pwd });
	wifi_task_add_withargs(&hwifi, wifi_tcpconn_args, wifi_autosetup_tcpconn_taskname, 2, (int[] ) { (int) wifi_conf_tcpip,
					(int) wifi_conf_tcpport });
	wifi_send_tasklist(wifi_autosetup_greet_buf, 0);
	// 怎么做TASK失败的条件跳转？你可以在后面设置一个程序状态字寄存器，这样看上去就像虚拟机了。
	// 我现在用的是TASK失败在回调函数里面处理的模式。
}

static char wifi_greet1_buf[60];
static const char *wifi_greet1_tasklist_name = "Greet 1";
static void wifi_greet1_tasklist() {
	if (wifi_task_isempty(&hwifi)) {
		logu_s(LOGU_INFO, "Start WiFi Greet 1 task list.");
		snprintf(wifi_greet1_buf, 60, "Another ten seconds passed on machine %hu ($ _ $)\r\n", machine_id);
		wifi_task_setlistname(&hwifi, wifi_greet1_tasklist_name);
		wifi_send_tasklist(wifi_greet1_buf, 0);
	}
}
void wifi_greet_1() {
	if (cycletick_everyms(10000)) {
		wifi_greet1_tasklist();
	}
}

static const char *wifi_send_tasklist_name = "Standalone send";
void wifi_send_tasklist(const char *str, int normalmode) {
	if (wifi_task_isempty(&hwifi) && wifi_task_getlistname(&hwifi) == NULL) {
		wifi_task_setlistname(&hwifi, wifi_send_tasklist_name);
	}
	wifi_task_add(&hwifi, wifi_checkat);
	if (normalmode) {
		// have a chance to fail
		wifi_task_add_withargs(&hwifi, wifi_send_normal_args, wifi_startsend_taskname, 2, (int[] ) { (int) str,
						(int) strlen(str) });
	} else {
		wifi_task_add(&hwifi, wifi_setmodetrans_unvarnished);
		wifi_task_add_withname(&hwifi, wifi_startsend_unvarnished, wifi_startsend_taskname);
		wifi_task_add_withargs(&hwifi, wifi_send_str_args, NULL, 1, (int[] ) { (int) str });
		wifi_task_add_withargs(&hwifi, wifi_task_delay, NULL, 1, (int[] ) { 100 });
		wifi_task_add(&hwifi, wifi_stopsend_unvarnished);
		wifi_task_add(&hwifi, wifi_setmodetrans_normal);
	}
	wifi_task_add(&hwifi, wifi_checkat);
}

static const char *wifi_check_connection_tasklist_name="check conn";
_Bool wifi_check_connection_tasklist() {
	if (wifi_task_isempty(&hwifi)) {
		logu_s(LOGU_INFO, "wifi check connection start");
		wifi_task_setlistname(&hwifi, wifi_check_connection_tasklist_name);
		wifi_task_add(&hwifi, wifi_checkconnection);
		return true;
	} else {
		return false;
	}
}

void wifi_tick_callback(Wifi_HandleTypeDef *phwifi, WifiRtnState state, int index, int finished) {
	if (finished && wifi_change_to_raw_when_success) {
		wifi_change_to_raw_when_success = 0;
		if(state == WRS_OK) {
			wifi_pipe_raw = 1;
		}
	}
	// 对于一些任务的完成，做个别变化处理。
	const char *tasksname = wifi_task_getlistname(phwifi);
	const char *itemname = NULL;
	if (index > -1)
		itemname = wifi_task_getitemname(phwifi, index);
	if (tasksname == wifi_autosetup_tasklist_name) {
		wifi_reconnecting = 0;
		if (state != WRS_OK) {
			logu_s(LOGU_ERROR, "WiFi Auto Setup task list failed");
			if (itemname == wifi_autosetup_at_check) {
				wifi_task_clear(phwifi);
				logu_s(LOGU_ERROR, "Failed on WiFi function check, check your hardware.");
			} else if (itemname == wifi_autosetup_joinap_taskname) {
				wifi_task_clear(phwifi);
				logu_s(LOGU_ERROR, "Failed on WiFi connect AP, check your router and WiFi SSID/PWD settings");
			} else if (itemname == wifi_autosetup_tcpconn_taskname) {
				wifi_task_clear(phwifi);
				logu_s(LOGU_ERROR, "Failed on TCP connect, check your TCP server and IP/Port settings");
			} else if (itemname == wifi_startsend_taskname) {
				wifi_task_clear(phwifi);
				logu_s(LOGU_ERROR, "Failed in changing to send mode, check your WiFi and TCP settings");
			} else {
				logu_s(LOGU_ERROR, "But that's a minor problem, WiFi Auto Setup will try to continue.");
			}
		}
	} else if (tasksname == wifi_greet1_tasklist_name) {
		if (state != WRS_OK) {
			logu_s(LOGU_ERROR, "WiFi Greet 1 task list failed");
			wifi_task_clear(phwifi);
			#if WIFI_RECONNECT_WHEN_GREET_FAILED
			if(wifi_reconnecting == 0) {
				wifi_reconnecting = 1;
				logu_s(LOGU_INFO, "Wifi reconnect issued.");
			}
			#endif
		}
	} else if (tasksname == wifi_send_tasklist_name) {
		if (state != WRS_OK) {
			logu_s(LOGU_ERROR, "WiFi send task list failed");
			wifi_task_clear(phwifi);
			#if WIFI_RECONNECT_WHEN_SEND_FAILED
			if(wifi_reconnecting == 0) {
				wifi_reconnecting = 1;
				logu_s(LOGU_INFO, "Wifi reconnect issued.");
			}
			#endif
		}
	} else if (tasksname == wifi_check_connection_tasklist_name) {
		if (state != WRS_OK) {
			logu_s(LOGU_ERROR, "WiFi check connection task list failed");
			wifi_task_clear(phwifi);
#if WIFI_RECONNECT_WHEN_CHECK_FAILED
			if(wifi_reconnecting == 0) {
				wifi_reconnecting = 1;
				logu_s(LOGU_INFO, "Wifi reconnect issued.");
			}
#endif
		}
	}
	logu_f(LOGU_DEBUG, "WiFi returns %s on index %d of %s.", rtntostr(state), index, tasksname);
	if (finished)
		logu_f(LOGU_INFO, "WiFi task list %s finished", tasksname);
}
#else
void wifi_send_tasklist(const char *str, int normalmode) {
}
#endif

HAL_StatusTypeDef cmd_can_send(struct cmd *cmd) {
	uint8_t msg[8] = {0};
	// msg[0] is used in can_io for store MSB of self machine id.
	msg[1] = (uint8_t)cmd->type;
	switch(cmd->type) {
		case cmd_motor_stop:
		case cmd_motor_speed:
		case cmd_motor_reset:
		case cmd_motor_set_home:
		case cmd_motor_pos:
		case cmd_motor_relapos:
		case cmd_motor_percent:
		case cmd_motor_where:
		msg[2] = (uint8_t)cmd->motorcmd.pos;
		msg[3] = (uint8_t)(cmd->motorcmd.pos >> 8);
		msg[4] = (uint8_t)(cmd->motorcmd.pos >> 16);
		msg[5] = (uint8_t)(cmd->motorcmd.pos >> 24);
		msg[6] = cmd->motorcmd.dir;
		break;
		case cmd_setting_id:
		msg[2] = (uint8_t)cmd->settingcmd.machine_id;
		msg[3] = (uint8_t)(cmd->settingcmd.machine_id >> 8);
		break;
		case cmd_setting_show_id:
		break;
		case cmd_setting_mcycle:
		break;
		case cmd_setting_lm_limit_in:
		case cmd_setting_lm_limit_out:
		msg[2] = (uint8_t)cmd->settingcmd.lm_limit_in;
		msg[3] = (uint8_t)cmd->settingcmd.lm_limit_in >> 8;
		msg[4] = (uint8_t)cmd->settingcmd.lm_limit_in >> 16;
		msg[5] = (uint8_t)cmd->settingcmd.lm_limit_in >> 24;
		break;
		case cmd_setting_load:
		break;
		case cmd_wifi_check:
		case cmd_wifi_auto:
		case cmd_wifi_join:
		case cmd_wifi_leave:
		case cmd_wifi_tcp_connect:
		case cmd_wifi_tcp_drop:
		case cmd_wifi_setap:
		case cmd_wifi_settcp:
		case cmd_wifi_dummy:
		break;
		case cmd_led_color_grad_to:
		msg[5] = (uint8_t)cmd->ledcmd.ms;
		msg[6] = (uint8_t)cmd->ledcmd.ms >> 8;
		// no break here
		case cmd_led_color:
		msg[2] = cmd->ledcmd.color.r;
		msg[3] = cmd->ledcmd.color.g;
		msg[4] = cmd->ledcmd.color.b;
		break;
		default:
		logu_f(LOGU_ERROR, "CAN Unknown send cmd: %d.", (int)cmd->type);
	}
	return can_send_cmd(msg, 8, cmd->receiver);
}

_Bool cmd_can_read(char* data, uint8_t len, uint16_t from, struct cmd* cmd) {
	cmd->sender = from;
	cmd->type = (enum cmdtype)data[1];
	switch(cmd->type) {
		case cmd_motor_stop:
		case cmd_motor_speed:
		case cmd_motor_reset:
		case cmd_motor_set_home:
		case cmd_motor_pos:
		case cmd_motor_relapos:
		case cmd_motor_percent:
		case cmd_motor_where:
		cmd->motorcmd.pos = data[2] + (data[3] << 8) + (data[4] << 16) + (data[5] << 24);
		cmd->motorcmd.dir = data[6];
		break;
		case cmd_setting_id:
		cmd->settingcmd.machine_id = data[2] + (data[3] << 8);
		break;
		case cmd_setting_show_id:
		break;
		case cmd_setting_mcycle:
		break;
		case cmd_setting_lm_limit_in:
		case cmd_setting_lm_limit_out:
		cmd->settingcmd.lm_limit_in = data[2] + (data[3] << 8) + (data[4] << 16) + (data[5] << 24);
		break;
		case cmd_setting_load:
		break;
		case cmd_wifi_check:
		case cmd_wifi_auto:
		case cmd_wifi_join:
		case cmd_wifi_leave:
		case cmd_wifi_tcp_connect:
		case cmd_wifi_tcp_drop:
		case cmd_wifi_setap:
		case cmd_wifi_settcp:
		case cmd_wifi_dummy:
		break;
		case cmd_led_color_grad_to:
		cmd->ledcmd.ms = data[5]  + (data[6] << 8);
		// no break here
		case cmd_led_color:
		cmd->ledcmd.color.r = data[2];
		cmd->ledcmd.color.g = data[3];
		cmd->ledcmd.color.b = data[4];
		break;
		default:
		logu_f(LOGU_ERROR, "CAN Unknown read cmd: %d.", (int)cmd->type);
		return 0;
	}
	return 1;
}

void cmd_can_isr(char *data, size_t len, uint16_t from, _Bool isbroadcast, struct lm_handle *plmhandle) {
	struct cmd cmd = {0};
	logu_s(LOGU_INFO, "CAN Receive a command.");
	_Bool ok = cmd_can_read(data, len, from, &cmd);
	if(ok)
		ok = cmd_action(&cmd, plmhandle);
	// when receive a broadcast command, machine will only report error.
	// when receive other commands, machine will report ok and error.
	if(!isbroadcast || !ok)
		can_send_reply(cmd.sender, ok);
}

// ----------
// 设置电机循环测试的时候使用的命令。
static void mcycle_append_cmd(struct lm_handle *plmh, uint32_t count);
void mcycle_cmd(struct lm_handle *plmh, uint32_t count) {
	if (lm_cycle) {
		mcycle_append_cmd(plmh, count);
	}
}

static void mcycle_append_cmd(struct lm_handle *plmh, uint32_t count) {
	// TODO 芯片如果出现移动到一半就认为自己移动完成的情况，是奇怪BUG的开始，这是因为芯片过热了，这个时候也需要考虑处理的对策。
	if (L6470_BUSY1()) {
		// 在运转到某个位置的途中会出现BUSY
		// 其他时候都不会有BUSY
		// TODO 因为现在POS状态下的校准HOME是关闭的，所以一旦移动向内的时候误差向内没有一个HOME校准，后面可能向内的误差累积都没有校准。
		// 这就会引发问题。
		lm_cycle_pause_count = count;
	} else if (lm_cycle_pause_at_full_cycle || diffu(lm_cycle_pause_count, count, CYCLE_LIMIT) > lm_cycle_step_pause / CYCLE_INTV) {
		struct lm_model lmmod = plmh->mod;
		// 这个if对应如果设置在每一步暂停一会儿，那么等待一段count计数
		if ((lmmod.state == lm_state_pos && lmmod.pos == lm_cycle_in) || lmmod.state == lm_state_stop
				|| lmmod.state == lm_state_speed) {
			// state pos + lm_cycle_in 是运转到了最里面刚停下的状态
			// state stop 是匀速模式碰到了CN1之后的结果
			// state speed 是进入了匀速模式
			if (cn1_pressed()) {
				// 当移动到最内侧CN1被按下。
				if (!lm_cycle_pause_at_full_cycle || diffu(lm_cycle_pause_count, count, CYCLE_LIMIT) > lm_cycle_step_pause / CYCLE_INTV) {
					// 向外移动
					logu_s(LOGU_INFO, "Move forward");
					lm_append_newcmd(plmh, lm_cmd_pos, lm_cycle_out, 0);
				}
			} else if (lmmod.state != lm_state_speed) {
				// 当POS移动完成，没有按下CN1的时候进入向内匀速运动的模式。
				lm_cycle_speed_count = count;
				logu_s(LOGU_INFO, "Finding Home");
				lm_append_newcmd(plmh, lm_cmd_speed, 10000, 1);
			} else {
				// 进入匀速模式但是没有到头的情况
				if (diffu(lm_cycle_speed_count, count, CYCLE_LIMIT) > 200) {
					// 太长时间没有到头说明点击进入了过热保护，这个时候等待一段时间再下命令会好。
					lm_cycle_speed_count = count;
					logu_s(LOGU_WARN, "Finding Home Again");
					lm_append_newcmd(plmh, lm_cmd_speed, 10000, 1);
				}
			}
		} else if (lmmod.state == lm_state_pos && lmmod.pos == lm_cycle_out) {
			// 当移动到最外侧
			logu_s(LOGU_INFO, "Move back");
			lm_append_newcmd(plmh, lm_cmd_pos, lm_cycle_in, 0);
		}
	}
}
