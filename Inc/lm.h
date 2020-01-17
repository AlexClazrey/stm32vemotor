#ifndef __L6M_H__
#define __L6M_H__

#include "stm32f1xx.h"

/*
 * This file provides an abstract model of L6470 motor state.
 * */

enum lm_state {
	lm_state_reset,
	lm_state_speed,
	lm_state_pos,
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
};

struct lm_cmd {
	enum lm_cmd_type type;
	int32_t pos_speed;
	uint8_t dir_hard;
};

int lm_commit(struct lm_cmd *cmd, struct lm_model *model);

#endif