#include "command.h"
#include "DSpin/dspin.h"
#include "util.h"
#include "cycletick.h"
#include "log_uart.h"
#include "inputbuf.h"

#include "can_io.h"
#include "led.h"
#include "main.h"

#if WIFI_ENABLE == 1
#include "wifi8266/wifi_8266_mod.h"
#endif

#include <string.h>
#include <stdio.h>

/* Configuration 设置项目 */
#include "config.h"

// 这个是机器编号，只能是一个字节。
extern uint8_t machine_id;

// 移动到比例位置的时候使用的范围。
extern const int lm_limit_out;
extern const int lm_limit_in;

// 电机循环测试配置
extern const int lm_cycle_out;
extern const int lm_cycle_in;
// 1的时候只有跑完一整个测试循环，点击在最里面的位置才会暂停一会儿，
// 0的时候在测试里的每一步都会暂停一会儿。
extern int lm_cycle_pause_at_full_cycle;
extern int lm_cycle_step_pause;

// CAN 命令参数
extern const uint8_t CAN_CMD_VER;
extern const uint8_t CAN_CMD_LM;
extern const uint8_t CAN_CMD_STRING;

#if WIFI_ENABLE == 1
// WIFI 连接设置
extern char *WIFI_SSID;
extern char *WIFI_PWD;
extern char *WIFI_TCP_IP;
extern uint16_t WIFI_TCP_PORT;

#endif

#ifndef UART_INPUT_BUF_SIZE 
#define UART_INPUT_BUF_SIZE 300
#endif

#ifndef cmd_length_limit 
#define cmd_length_limit 40
#endif

#if WIFI_ENABLE == 1
// WiFi
// 这里写得丑了一点强行引用
extern UART_HandleTypeDef huart2;
static Wifi_HandleTypeDef hwifi = { .huart = &huart2 };
static int wifi_change_to_raw_when_success = 0;
static int wifi_pipe_raw = 0;
#endif

// motor cycle test
static int lm_cycle = 0; // 循环测试开启指示
static uint32_t lm_cycle_pause_count = 0; // 暂停的等待计数
static uint32_t lm_cycle_speed_count = 0; // 在过热无反应时候的等待计数

enum cmdfrom {
	CMD_FROM_SERIAL, CMD_FROM_WIFI,
};

/* Functions */
// ------------- Input Buffer Process
static void input_feedback(enum cmdfrom from, int success);
static enum inputcmdtype cmd_read_act(const char *src, struct lm_handle *plmh, enum cmdfrom from, int suppress_error);
static enum inputcmdtype cmd_parse(const char *cmd, struct lm_cmd *out_store, uint8_t *out_recevier);
static HAL_StatusTypeDef can_send_feedback(enum cmdfrom from, HAL_StatusTypeDef send_res);

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
#endif
		while (cycletick_now() < 8 && lineend) {
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

static enum inputcmdtype cmd_read_act(const char *src, struct lm_handle *plmh, enum cmdfrom from, int suppress_error) {
	struct lm_cmd cmd = { 0 };
	uint8_t receiver = 0;
	enum inputcmdtype type = cmd_parse(src, &cmd, &receiver);
	if (type == input_error) {
		// Error details was printed in parse function
		if (!suppress_error) {
			input_feedback(from, false);
		}
	} else if (type == input_empty) {
		// 读到empty表示输入缓冲区已经读完了。
	} else if (type == input_lmcmd) {
		// 如果是本机的命令，那么加入cmd queue。
		lm_append_cmd(plmh, &cmd);
		input_feedback(from, true);
	} else if (type == input_settings) {
		input_feedback(from, true);
	} else if (type == input_can) {
		HAL_StatusTypeDef hs = can_cmd_send(&cmd, receiver);
		can_send_feedback(from, hs);
		HAL_Delay(1); // TODO 这里强行停顿不太好，但是CAN SEND需要一点时间。
		input_feedback(from, hs == HAL_OK);
	} else if (type == input_wifi) {
		input_feedback(from, true);
	} else if (type == input_next) {
		// do nothing here
	} else {
		logu_s(LOGU_ERROR, "Unknown error [01]");
	}
	return type;
}

static void wifi_send_tasklist(const char *str, int normalmode);
static void input_feedback(enum cmdfrom from, int success) {
	if (from == CMD_FROM_SERIAL) {
		if (success) {
			logu_raw("<OK>\r\n", 6);
		} else {
			logu_raw("<FAIL>\r\n", 8);
		}
	} else if (from == CMD_FROM_WIFI) {
		if (success) {
			wifi_send_tasklist("<OK>\r\n", 0);
		} else {
			wifi_send_tasklist("<FAIL>\r\n", 0);
		}
	}
}

static HAL_StatusTypeDef can_send_feedback(enum cmdfrom from, HAL_StatusTypeDef send_res) {
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

static int cmd_parse_body(const char *cmd, struct lm_cmd *store);
static int read_mr_args(const char *cmd, uint32_t *out_speed, uint32_t *out_dir);
static enum inputcmdtype cmd_parse(const char *cmd, struct lm_cmd *out_store, uint8_t *out_receiver) {
	uint16_t receiver = (uint16_t) -1;

	logu_f(LOGU_TRACE, "Parse: %s", cmd);

	// 为了兼容之前的格式
	if (strncmp(cmd, "sig ", 4) == 0) {
		cmd += 4;
	}

	// if command has an receiver
	if (cmd[0] == '#') {
		size_t scanlen;
		if (sscanf(++cmd, "%hu%n", &receiver, &scanlen) != 1 || receiver > 255) {
			logu_s(LOGU_ERROR, "Cmd id read error.");
			return 1;
		}
		*out_receiver = (uint8_t) receiver;
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
	} else if (res == 2) {
		return input_settings;
	} else if (res == 0) {
		if (receiver == machine_id || receiver == (uint16_t) -1) {
			return input_lmcmd;
		} else {
			return input_can;
		}
	} else if (res == 3) {
		return input_wifi;
	} else {
		logu_f(LOGU_ERROR, "Unknown error [02]");
		return input_error;
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
static int cmd_parse_body(const char *cmd, struct lm_cmd *store) {
	// any input will turn off motor cycle test
	if (lm_cycle) {
		logu_f(LOGU_INFO, "Cycle off.");
		lm_cycle = 0;
	}
	if (cmd[0] == 'm') {
		if (strncmp(cmd + 1, "cycle", 5) == 0) {
			// TODO 把循环测试也做成能用CAN发送的命令。
			// TODO settings command 包括 mid 和 mcycle 都没有储存机制
			lm_cycle = !lm_cycle;
			if (lm_cycle) {
				// cycle kick start action, move to cycle_in position
				store->type = lm_cmd_pos;
				store->pos_speed = lm_cycle_in;
				return 0;
			}
			return 2;
		} else if (strncmp(cmd + 1, "home", 4) == 0) {
			store->type = lm_cmd_set_home;
		} else if (strncmp(cmd + 1, "reset", 5) == 0) {
			store->type = lm_cmd_reset;
		} else if (strncmp(cmd + 1, "id ", 3) == 0) {
			uint16_t id;
			if (sscanf(cmd + 4, "%hu", &id) == 1 && id < 256) {
				machine_id = id;
				return 2;
			} else {
				logu_s(LOGU_ERROR, "Read Machine Id Error, should between 0 and 255.");
				return 1;
			}
		} else if (cmd[1] == 'r' && cmd[2] == ' ') {
			// 这里一定要注意检验 cmd[2] 是空格，不然可能是NUL字符，就是一定要连续检验，想不到还有这个隐患。
			uint32_t pos, dir;
			if (read_mr_args(cmd + 3, &pos, &dir) != 2) {
				logu_s(LOGU_ERROR, "Read mr command failed.");
				return 1;
			}
			store->pos_speed = pos;
			store->dir_hard = dir;
			store->type = lm_cmd_speed;
		} else if (cmd[1] == 's') {
			store->type = lm_cmd_stop;
		} else if (cmd[1] == 'p') {
			if (cmd[2] == ' ') {
				if (sscanf(cmd + 3, "%ld", &(store->pos_speed)) != 1) {
					logu_s(LOGU_ERROR, "Read mp command failed.");
					return 1;
				}
			} else if (cmd[2] == 'p' && cmd[3] == ' ') {
				uint16_t percent;
				if (sscanf(cmd + 4, "%hu", &percent) != 1) {
					logu_s(LOGU_ERROR, "Read mpp percent failed.");
					return 1;
				}
				if (percent > 100) {
					logu_s(LOGU_ERROR, "Mpp percent should between 0 and 100.");
					return 1;
				}
				store->pos_speed = (int) ((double) percent / 100 * (lm_limit_out - lm_limit_in) + lm_limit_in);
			}
			store->type = lm_cmd_pos;
		} else {
			logu_s(LOGU_ERROR, "Command parse failed");
			return 1;
		}
	} else if (cmd[0] == 'w') {
#if WIFI_ENABLE==1
		// TODO 在 wifi 的 task 没有完全释放的时候不要执行下一个命令
		// 应该向上Log wifi busy这句话
		if (strncmp(cmd + 1, "at", 3) == 0) {
			wifi_task_add(&hwifi, wifi_checkat);
		} else if (strncmp(cmd + 1, "auto", 5) == 0) {
			// set up wifi
			wifi_autosetup_tasklist();
		} else if (strncmp(cmd + 1, "join", 4) == 0) {
			// join ap
			if (cmd[1 + 4] == '\0') {
				wifi_task_add_withargs(&hwifi, wifi_joinap_args, NULL, 2, (int[] ) { (int) WIFI_SSID, (int) WIFI_PWD });
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
			// tcp connect
			if (cmd[1 + 3] == '\0') {
				wifi_task_add_withargs(&hwifi, wifi_tcpconn_args, NULL, 2, (int[] ) { (int) WIFI_TCP_IP,
								(int) WIFI_TCP_PORT });
			} else {
				if (strlen(cmd) > 64) {
					logu_s(LOGU_ERROR, "IP and port are too long.");
					return 1;
				}
				uint16_t wifiscanport;
				int scnt = sscanf(cmd + 4, wifiscanip, &wifiscanport);
				if (scnt != 2) {
					logu_s(LOGU_ERROR, "IP or port parsed failed.");
					return 1;
				}
				wifi_task_add_withargs(&hwifi, wifi_tcpconn_args, NULL, 2, (int[] ) { (int) wifiscanip,
								(int) wifiscanport });
			}
		} else if (strncmp(cmd + 1, "send", 5) == 0) {
			// start send and set serial input into raw mode
			wifi_task_add(&hwifi, wifi_startsend_unvarnished);
			wifi_change_to_raw_when_success = 1;
			logu_s(LOGU_WARN, "Start wifi pipe mode.");
		} else if (strncmp(cmd + 1, "drop", 5) == 0) {
			// tcp disconnect
			wifi_task_add(&hwifi, wifi_dropsingleconn);
		} else if (strncmp(cmd + 1, "leave", 6) == 0) {
			// leave ap
			wifi_task_add(&hwifi, wifi_leaveap);
		} else {
			return 1;
		}
		return 3;
#else
		logu_s(LOGU_ERROR, "Command parse failed, Wifi module is disabled.");
		return 1;
#endif

	} else if (strncmp(cmd, "led", 3) == 0) {
		uint16_t r, g, b;
		int scnt = sscanf(cmd + 3, "%hu%hu%hu", &r, &g, &b);
		if(scnt != 3) {
			logu_s(LOGU_ERROR, "Led brightness parse failed.");
			return 1;
		}
		if(r > 255 || g > 255 || b > 255) {
			logu_s(LOGU_ERROR, "Led brightness too high.");
			return 1;
		}
		led_set((uint8_t)r, (uint8_t)g, (uint8_t)b);
		return 2;
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
void wifi_parse_cmd(struct lm_handle *plmh) {
	const char *cmd = wifi_rx_cap(&hwifi);
	if (cmd == NULL)
		return;
	const char *ipdstr = strstr(cmd, "\r\n+IPD,");
	if (ipdstr != NULL) {
		const char *s1 = strchr(ipdstr, ':');
		if (s1 == NULL) {
			logu_f(LOGU_ERROR, "WiFi received an invalid data: %s", cmd);
			return;
		}
		int startindex = s1 - cmd + 1;
		enum inputcmdtype type = cmd_read_act(cmd + startindex, plmh, CMD_FROM_WIFI, 1);
		if (type != input_empty)
			logu_f(LOGU_DEBUG, "WiFi received command type: %d", (int) type);
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
	// 6. JOIN AP
	// 7. CONNECT TCP
	// 8. Start send mode
	// 9. Send Hello
	// 10. delay
	// 11. Exit send mode
	// 12. check "AT"
	logu_s(LOGU_INFO, "Start WiFi Auto Setup task list.");
	snprintf(wifi_autosetup_greet_buf, 50, "Machine %hu greetings ($ a $).\r\n", (uint16_t) machine_id);
	wifi_task_setlistname(&hwifi, wifi_autosetup_tasklist_name);
	wifi_task_add(&hwifi, wifi_stopsend_unvarnished);
	wifi_task_add(&hwifi, wifi_checkat);
	wifi_task_add_withname(&hwifi, wifi_checkat, wifi_autosetup_at_check);
	wifi_task_add(&hwifi, wifi_setmodewifi_client);
	wifi_task_add(&hwifi, wifi_setsingleconn);
	wifi_task_add(&hwifi, wifi_setmodetrans_normal);
	wifi_task_add_withargs(&hwifi, wifi_joinap_args, wifi_autosetup_joinap_taskname, 2, (int[] ) { (int) WIFI_SSID,
					(int) WIFI_PWD });
	wifi_task_add_withargs(&hwifi, wifi_tcpconn_args, wifi_autosetup_tcpconn_taskname, 2, (int[] ) { (int) WIFI_TCP_IP,
					(int) WIFI_TCP_PORT });
	wifi_send_tasklist(wifi_autosetup_greet_buf, 0);
	// 怎么做TASK失败的条件跳转？你可以在后面设置一个程序状态字寄存器，这样看上去就像虚拟机了。
	// 我现在用的是TASK失败在回调函数里面处理的模式。
}

static char wifi_greet1_buf[60];
static const char *wifi_greet1_tasklist_name = "Greet 1";
static void wifi_greet1_tasklist() {
	if (wifi_task_isempty(&hwifi)) {
		logu_s(LOGU_INFO, "Start WiFi Greet 1 task list.");
		snprintf(wifi_greet1_buf, 60, "Another ten seconds passed on machine %hu ($ _ $)\r\n", (uint16_t) machine_id);
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
static void wifi_send_tasklist(const char *str, int normalmode) {
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

void wifi_tick_callback(Wifi_HandleTypeDef *phwifi, WifiRtnState state, int index, int finished) {
	if (finished && state == WRS_OK && wifi_change_to_raw_when_success) {
		wifi_pipe_raw = 1;
		wifi_change_to_raw_when_success = 0;
	}
	// 对于一些任务的完成，做个别变化处理。
	const char *tasksname = wifi_task_getlistname(phwifi);
	const char *itemname = NULL;
	if (index > -1)
		itemname = wifi_task_getitemname(phwifi, index);
	if (tasksname == wifi_autosetup_tasklist_name) {
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
		}
	} else if (tasksname == wifi_send_tasklist_name) {
		if (state != WRS_OK) {
			logu_s(LOGU_ERROR, "WiFi send task list failed");
			wifi_task_clear(phwifi);
		}
	}
	logu_f(LOGU_DEBUG, "WiFi returns %s on index %d of %s.", rtntostr(state), index, tasksname);
	if (finished)
		logu_f(LOGU_INFO, "WiFi task list %s finished", tasksname);
}
#else
static void wifi_send_tasklist(const char *str, int normalmode) {
}
#endif

// CAN send lm_cmd
HAL_StatusTypeDef can_cmd_send(struct lm_cmd *cmd, uint8_t receiver_id) {
	uint8_t msg[8];
	uint32_t pos = cmd->pos_speed;
	msg[0] = CAN_CMD_VER;
	msg[1] = CAN_CMD_LM;
	msg[2] = receiver_id;
	msg[3] = (uint8_t) (cmd->type);
	msg[4] = (uint8_t) (pos >> 16);
	msg[5] = (uint8_t) (pos >> 8);
	msg[6] = (uint8_t) (pos);
	msg[7] = (uint8_t) (cmd->dir_hard);
	return can_msg_add(msg, 8);
}

/*
 * return 0 for buffer empty
 * return -1 for received invalid
 * return 1 for lm_cmd got.
 * return 2 for string got.
 */
int canbuf_read(struct lm_cmd *dest, char *data, size_t len) {
	if (len > 0) {
		if (len >= 3) {
			uint8_t ver = data[0], cmd = data[1], id = data[2];
			if (id == machine_id) {
				if (ver != CAN_CMD_VER) {
					logu_f(LOGU_ERROR, "CAN Message version doesn't equal mine.");
					return -1;
				}
				if (cmd == CAN_CMD_LM) {
					// 按照 cmd_run 里面的编码方法重新组装消息
					dest->type = data[3];
					dest->pos_speed = (int32_t) ((data[4] << 16) + (data[5] << 8) + (data[6]));
					dest->dir_hard = data[7];
					return 1;
				} else if (cmd == CAN_CMD_STRING) {
					char buf[8] = { 0 };
					memcpy(buf, data + 2, len - 2);
					logu_f(LOGU_INFO, "CAN Received: %s", buf);
					return 2;
				} else {
					logu_f(LOGU_ERROR, "CAN unknown command %hu.", (uint16_t) cmd);
					return -1;
				}
			} else {
				logu_f(LOGU_DEBUG, "Ignore CAN Message with other's id #%hu", (uint16_t) id);
				return -1;
			}
		} else {
			logu_f(LOGU_WARN, "CAN Invalid Message: Too Short.");
			return -1;
		}
	} else {
		return 0;
	}
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
		// TODO 因为现在POS状态下的校准HOME是关闭的（在之后的注释里），所以一旦移动向内的时候误差向内没有一个HOME校准，后面可能向内的误差累积都没有校准。
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
