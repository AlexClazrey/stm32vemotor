#ifndef __L6470M_H__
#define __L6470M_H__

#include "stm32f1xx.h"

/*
 * This file provides an abstract model of L6470 motor state.
 * */

enum lm_state {
	lm_state_reset,
	lm_state_speed,
	lm_state_pos,
	lm_state_relapos,
	lm_state_stop,
};

struct lm_model {
	enum lm_state state;
	int32_t pos;
	uint32_t speed;
	uint32_t dir;
};

enum lm_cmd_type {
	lm_cmd_empty,
	lm_cmd_reset,
	lm_cmd_set_home,
	lm_cmd_stop,
	lm_cmd_speed,
	lm_cmd_pos,
	lm_cmd_relapos,
	lm_cmd_where,
};

struct lm_cmd {
	enum lm_cmd_type type;
	int32_t pos_speed;
	uint8_t dir_hard;
};

#define lm_handle_queue_len 10
struct lm_handle {
	volatile struct lm_model mod;  // 如果做了状态变化的中断反馈的话需要volatile
	struct lm_cmd queue[lm_handle_queue_len];
	uint32_t head;
	uint32_t tail;
};

void lm_init(struct lm_handle *handle);
void lm_tick(struct lm_handle* handle);
int lm_hasspace(struct lm_handle *handle);
int lm_append_cmd(struct lm_handle* handle, struct lm_cmd *cmd);
int lm_append_newcmd(struct lm_handle* handle, enum lm_cmd_type type, int32_t pos_speed, uint8_t dir_hard);
struct lm_cmd* lm_first(struct lm_handle *handle);
struct lm_cmd* lm_pop(struct lm_handle *handle);
int lm_commit(struct lm_handle *handle);
int lm_read_pos();

#endif
