#include "lm.h"
#include "util.h"
#include "DSpin/dspin.h"
#include "log_uart.h"
#include <string.h>

// import break pins
#include "main.h"

static int lm_commit_cmd(struct lm_cmd *cmd, volatile struct lm_model *model);

void lm_init(struct lm_handle *handle) {
	memset(handle, 0, sizeof(struct lm_handle));
}

static void lm_breakoff() {
	Break1_GPIO_Port->ODR |= Break1_Pin;
	Break2_GPIO_Port->ODR |= Break2_Pin;
}
static void lm_breakon() {
	Break1_GPIO_Port->ODR &= ~Break1_Pin;
	Break2_GPIO_Port->ODR &= ~Break2_Pin;
}

void lm_tick(struct lm_handle *handle) {
	handle->mod.pos = lm_read_pos();
	if (handle->mod.state == lm_state_speed) {
		lm_breakoff();
	} else if ((handle->mod.state == lm_state_relapos || handle->mod.state == lm_state_relapos) && L6470_BUSY1()) {
		lm_breakoff();
	} else {
		lm_breakon();
	}
}

int lm_hasspace(struct lm_handle *handle) {
	if (handle == NULL)
		return 0;
	uint32_t diff = diffu(handle->tail, handle->head, lm_handle_queue_len);
	return diff != 1;
}

int lm_append_cmd(struct lm_handle *handle, struct lm_cmd *cmd) {
	if (lm_hasspace(handle)) {
		handle->queue[handle->tail] = *cmd;
		handle->tail = addu(handle->tail, 1, lm_handle_queue_len);
		return 1;
	} else {
		return 0;
	}
}

int lm_append_newcmd(struct lm_handle *handle, enum lm_cmd_type type, int32_t pos_speed, uint8_t dir_hard) {
	if (lm_hasspace(handle)) {
		struct lm_cmd *pcmd = handle->queue + handle->tail;
		pcmd->dir_hard = dir_hard;
		pcmd->pos_speed = pos_speed;
		pcmd->type = type;
		handle->tail = addu(handle->tail, 1, lm_handle_queue_len);
		return 1;
	} else {
		return 0;
	}
}

struct lm_cmd* lm_first(struct lm_handle *handle) {
	if (handle == NULL)
		return NULL;
	if (handle->head >= lm_handle_queue_len) {
		logu_f(LOGU_ERROR, "LM cmd queue head out of range");
		handle->head = 0;
		return NULL;
	}
	if (handle->head == handle->tail) {
		return NULL;
	}
	return handle->queue + handle->head;
}

struct lm_cmd* lm_pop(struct lm_handle *handle) {
	struct lm_cmd *res = lm_first(handle);
	if (res == NULL) {
		return NULL;
	}
	handle->head = addu(handle->head, 1, lm_handle_queue_len);
	return res;
}

int lm_commit(struct lm_handle *handle) {
	int res = lm_commit_cmd(lm_first(handle), &handle->mod);
	lm_pop(handle);
	return res;
}

/*
 * Return 1 for success, 0 for failed.
 * */
static int lm_commit_cmd(struct lm_cmd *cmd, volatile struct lm_model *model) {
	if (cmd == NULL || model == NULL) {
		logu_f(LOGU_ERROR, "LM lm_commit got null pointer.");
		return 0;
	}
	if (cmd->type == lm_cmd_empty) {
		return 1;
	} else if (cmd->type == lm_cmd_reset) {
		dSPIN_Reset_Device();
		logu_f(LOGU_TRACE, "LM chip reset.");
		model->state = lm_state_reset;
		model->pos = 0;
		model->speed = 0;
		model->dir = 0;
	} else if (cmd->type == lm_cmd_set_home) {
		dSPIN_Reset_Pos();
		logu_f(LOGU_TRACE, "LM home position set.");
		model->pos = 0;
	} else if (cmd->type == lm_cmd_stop) {
		if (cmd->dir_hard) {
			dSPIN_Hard_Stop();
			logu_f(LOGU_TRACE, "LM Hard Stop Commit");
		} else {
			dSPIN_Soft_Stop();
			logu_f(LOGU_TRACE, "LM Soft Stop Commit");
		}
		model->state = lm_state_stop;
	} else if (cmd->type == lm_cmd_speed) {
		dSPIN_Run(cmd->dir_hard, cmd->pos_speed);
		logu_f(LOGU_TRACE, "LM speed: %ld, dir: %lu", cmd->pos_speed, (uint32_t) cmd->dir_hard);
		model->state = lm_state_speed;
		model->speed = cmd->pos_speed;
		model->dir = cmd->dir_hard;
	} else if (cmd->type == lm_cmd_pos) {
		// stop previous move or it won't listen to another command
		if (L6470_BUSY1()) {
			dSPIN_Hard_Stop();
			logu_f(LOGU_TRACE, "Stop previous movement.");
		}
		logu_f(LOGU_INFO, "pos: %ld", cmd->pos_speed);
		dSPIN_Go_To(cmd->pos_speed);
		logu_f(LOGU_TRACE, "LM pos: %ld", cmd->pos_speed);
		model->state = lm_state_pos;
		model->speed = 0;
		model->pos = cmd->pos_speed;
	} else if (cmd->type == lm_cmd_relapos) {
		if (L6470_BUSY1()) {
			dSPIN_Hard_Stop();
			logu_f(LOGU_TRACE, "Stop previous movement.");
		}
		uint32_t step = ABS(cmd->pos_speed);
		dSPIN_Move(cmd->pos_speed > 0, step);
		logu_f(LOGU_TRACE, "LM move: %ld", cmd->pos_speed);
		model->dir = cmd->pos_speed > 0;
		model->state = lm_state_relapos;
	} else if (cmd->type == lm_cmd_where) {
		int32_t where = dSPIN_Get_Pos();
		logu_f(LOGU_INFO, "Motor is now at: %d.", where);
	}
	return 1;
}

int lm_read_pos() {
	return dSPIN_Get_Pos();
}
