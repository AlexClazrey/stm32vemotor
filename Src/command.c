#include "command.h"
#include "util.h"
#include "cycletick.h"
#include "log_uart.h"
#include "can_io.h"
#include "DSpin/dspin.h"
#include "wifi8266/wifi_8266_mod.h"
#include "main.h"
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

// CAN 命令参数
extern const uint8_t CAN_CMD_VER;
extern const uint8_t CAN_CMD_LM;
extern const uint8_t CAN_CMD_STRING;


#ifndef UART_INPUT_BUF_SIZE 
#define UART_INPUT_BUF_SIZE 300
#endif

#ifndef cmd_length_limit 
#define cmd_length_limit 40
#endif

// 引用主循环的配置
extern const uint32_t COUNT_LIMIT;


/* Runtime Variables */
// Serial input command
static char inputbuf[UART_INPUT_BUF_SIZE] = { 0 }; // 这个不需要 volatile 因为在读取这段的过程里读取的内容不会被更新改变，所以可以缓存。
static int inputstart = 0; // 现在中断程序DMA不检测是不是覆盖，所以不需要volatile了
static volatile int inputend = 0;

// cmd parse buffer
static char cmdbuf[cmd_length_limit + 1] = { 0 }; // 这个只会在主循环里面parse，所以不需要volatile

// WiFi
// 这里写得丑了一点强行引用
extern UART_HandleTypeDef huart2;
static Wifi_HandleTypeDef hwifi = {.huart = &huart2};

// motor cycle test
static int lm_cycle = 0; // 循环测试开启指示
static uint32_t lm_cycle_pause_count = 0; // 暂停的等待计数
static uint32_t lm_cycle_speed_count = 0; // 在过热无反应时候的等待计数

// ------------- Input Buffer Process
static void input_feedback(int success);
static enum inputcmdtype inputbuf_read_one_cmd(struct lm_cmd *out_pcmd, uint8_t *out_receiver);
static int cmd_copy(char *dest, char *buf, int start, int end, int bufsize, int cmdsize);
static enum inputcmdtype cmd_parse(char *cmd, struct lm_cmd *out_store, uint8_t *out_recevier);
static HAL_StatusTypeDef can_send_log(HAL_StatusTypeDef send_res);

void inputbuf_read(struct lm_handle *plmh) {
	// 这个函数试把解析InputBuffer的工作移动到主循环里面，这样可以面对大批量的命令汇入
	// 首先先向后寻找\r\n或者\0的位置，复制到cmdbuf里面，然后清理cmdbuf，然后解析。
	// 为了能在一次主循环里面处理多条命令。
	// 需要创建不止一个命令缓存而是很多个命令缓存池
	// 如果命令缓存池还有一个空位那就尝试读取
	// 因为不是自己的命令会沿着CAN发送出去不会占用空位
	// 如果是自己的命令那么占用一个位置
	// 如过缓存池被用满了那么停止这里的Read Input同时应该发出一个警告。
	// 发送很多个CAN指令也可能消耗很多时间，所以这里还需要在Parse一个新的命令之前注意检查时间，
	// 如果时间不多了那么就留到下一个循环处理。
	// 我也需要log一下处理一条命令需要用多少时间。
	// copy to cmd buffer and parse
	//
	while (cycle_tick_now() < 8) {
		if (!lm_hasspace(plmh)) {
			logu_s(LOGU_WARN, "Stop reading input due to full cmd queue");
			return;
		}
		struct lm_cmd cmd = { 0 };
		uint8_t receiver = 0;
		enum inputcmdtype type = inputbuf_read_one_cmd(&cmd, &receiver);

		if (type == input_error) {
			// Error details was printed in parse function
			input_feedback(false);
		} else if (type == input_empty) {
			// 读到empty表示输入缓冲区已经读完了。
			break;
		} else if (type == input_lmcmd) {
			// 如果是本机的命令，那么加入cmd queue。
			lm_append_cmd(plmh, &cmd);
			input_feedback(true);
		} else if (type == input_settings) {
			input_feedback(true);
		} else if (type == input_can) {
			HAL_StatusTypeDef hs = can_cmd_send(&cmd, receiver);
			can_send_log(hs);
			input_feedback(hs == HAL_OK);
		} else if (type == input_next) {
			// do nothing here
		} else {
			logu_s(LOGU_ERROR, "Unknown error [01]");
		}
	}

}

char* inputbuf_get() {
	return inputbuf;
}

void inputbuf_setend(uint32_t end) {
	inputend = end;
}

static enum inputcmdtype inputbuf_read_one_cmd(struct lm_cmd *out_pcmd, uint8_t *out_receiver) {
	uint32_t cursor = inputstart;
	int triggered = 0;
	while (cursor != (uint32_t) inputend) {
		// check char here
		char ch = inputbuf[cursor];
		if (ch == '\0' || ch == '\r' || ch == '\n') {
			triggered = 1;
			break;
		}
		cursor++;
		if (cursor == UART_INPUT_BUF_SIZE) {
			cursor = 0;
		}
	}
	if (!triggered) {
		return input_empty;
	}

	if (cursor == (uint32_t) inputstart) {
		logu_f(LOGU_DEBUG, "Yet another line ending.");
		inputstart = addu(cursor, 1, UART_INPUT_BUF_SIZE);
		return input_next; // 现在如果 \r\n 就会触发一次 next
	}

	int res = cmd_copy(cmdbuf, inputbuf, inputstart, cursor, UART_INPUT_BUF_SIZE, cmd_length_limit);
	// 这个时候cursor指向的是 \r\n\0 不可能是 inputend 所以要再加一
	inputstart = addu(cursor, 1, UART_INPUT_BUF_SIZE);

	if (res) {
		return cmd_parse(cmdbuf, out_pcmd, out_receiver);
	} else {
		return input_error;
	}
}

static void input_feedback(int success) {
	if (success) {
		logu_raw("<OK>\r\n", 6);
	} else {
		logu_raw("<FAIL>\r\n", 8);
	}
}

static HAL_StatusTypeDef can_send_log(HAL_StatusTypeDef send_res) {
	if (send_res == HAL_OK) {
		logu_f(LOGU_DEBUG, "CAN Sent.");
	} else {
		logu_f(LOGU_ERROR, "CAN Send failed.");
	}
	return send_res;
}

// --------- Command

/* Command format is:
 * [#<id>](mr|ms|mp|mpp|mreset|mhome|mcycle|mid)[ <args>]
 */

// return 1 is good, 0 is error.
static int cmd_copy(char *dest, char *buf, int start, int end, int bufsize, int cmdsize) {
	int len = cycarrtoarr(dest, cmdsize, buf, start, end, bufsize);
	// 这里要记得 cstr 末尾的 NUL 字符
	if (len != -1) {
		dest[len] = '\0';
		return 1;
	} else {
		logu_f(LOGU_ERROR, "Input longer than cmd length limit.");
		return 0;
	}
}

static int cmd_parse_body(char *cmd, struct lm_cmd *store);
static int read_mr_args(char *cmd, uint32_t *out_speed, uint32_t *out_dir);
static enum inputcmdtype cmd_parse(char *cmd, struct lm_cmd *out_store, uint8_t *out_receiver) {
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
	} else {
		logu_f(LOGU_ERROR, "Unknown error [02]");
		return input_error;
	}
}

// return 0: lm_cmd command, command which actually modifies lm_cmd* store.
// return 1: parse failed
// return 2: settings command
// return 3: wifi command
static int cmd_parse_body(char *cmd, struct lm_cmd *store) {
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
	} else if(cmd[0] == 'w') {
		// TODO 在wifi的callstack没有完全释放的时候不要执行下一个命令
		// 应该向上Log wifi busy这句话
		if(strncmp(cmd+1, "setup", 5) == 0) {
			// set up wifi
			// 1. check "AT" twice (to remove noise input)
			// 2. AT+CWMODE=1 setmodeclient()
			// 3. AT+CIPMUX=0 setsingleconn()
			// 4. AT+CIPMODE=1 setmodetrans()

		} else if(strncmp(cmd+1, "join", 4) == 0) {
			// join ap
			//
		} else if(strncmp(cmd+1, "tcp", 3) == 0) {
			// tcp connect
		} else if(strncmp(cmd+1, "trans", 5) == 0) {
			// set transparent

		} else if(strncmp(cmd+1, "dis", 3) == 0) {
			// tcp disconnect
		} else if(strncmp(cmd+1, "leave", 5) == 0) {
			// leave ap
		}
	} else {
		logu_s(LOGU_ERROR, "Command parse failed");
		return 1;
	}
	return 0;
}

static int read_mr_args(char *cmd, uint32_t *out_speed, uint32_t *out_dir) {
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

// WiFi receive
// TODO 当Wifi进入直传模式之后，先用一个缓冲区收集输入，然后在主循环里面调取解析函数
// 解析函数可以使用



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
	} else if (lm_cycle_pause_at_full_cycle || diffu(lm_cycle_pause_count, count, COUNT_LIMIT) > 100) {
		struct lm_model lmmod = plmh->mod;
		// 这个if对应如果设置在每一步暂停一会儿，那么等待一段count计数
		if ((lmmod.state == lm_state_pos && lmmod.pos == lm_cycle_in) || lmmod.state == lm_state_stop
				|| lmmod.state == lm_state_speed) {
			// state pos + lm_cycle_in 是运转到了最里面刚停下的状态
			// state stop 是匀速模式碰到了CN1之后的结果
			// state speed 是进入了匀速模式
			if (cn1_pressed()) {
				// 当移动到最内侧CN1被按下。
				if (!lm_cycle_pause_at_full_cycle || diffu(lm_cycle_pause_count, count, COUNT_LIMIT) > 100) {
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
				if (diffu(lm_cycle_speed_count, count, COUNT_LIMIT) > 200) {
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
