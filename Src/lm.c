#include "lm.h"
#include "util.h"
#include "./DSpin/dspin.h"
#include "log_uart.h"
#include <string.h>


static int lm_commit_cmd(struct lm_cmd *cmd, volatile struct lm_model *model);

void lm_init(struct lm_handle *handle) {
	memset(handle, 0, sizeof(struct lm_handle));
}

int lm_hasspace(struct lm_handle *handle) {
	if(handle == NULL)
		return 0;
	return diffu(handle->tail, handle->head, lm_handle_queue_len) > 1;
}

int lm_append_cmd(struct lm_handle *handle, struct lm_cmd *cmd) {
	if(lm_hasspace(handle)) {
		handle->queue[handle->tail] = *cmd;
		handle->tail = addu(handle->tail, 1, lm_handle_queue_len);
		return 1;
	} else {
		return 0;
	}
}

int lm_append_newcmd(struct lm_handle *handle, enum lm_cmd_type type, int32_t pos_speed, uint8_t dir_hard) {
	if(lm_hasspace(handle)) {
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

struct lm_cmd* lm_first(struct lm_handle* handle) {
	if(handle == NULL)
		return NULL;
	if(handle->head >= lm_handle_queue_len) {
		log_uartf(LOGERROR, "LM cmd queue head out of range");
		handle->head = 0;
		return NULL;
	}
	if(handle->head == handle->tail) {
		return NULL;
	}
	return handle->queue + handle->head;
}

struct lm_cmd* lm_pop(struct lm_handle *handle) {
	struct lm_cmd* res = lm_first(handle);
	if(res == NULL) {
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
	if(cmd == NULL || model == NULL) {
		log_uartf(LOGERROR, "LM lm_commit got null pointer.");
		return 0;
	}
	if(cmd->type == lm_cmd_empty) {
		return 1;
	} else if(cmd->type == lm_cmd_reset) {
		dSPIN_Reset_Device();
		log_uartf(LOGDEBUG, "LM chip reset.");
		model->state = lm_state_reset;
		model->pos = 0;
		model->speed = 0;
		model->dir = 0;
	} else if(cmd->type == lm_cmd_set_home) {
		dSPIN_Reset_Pos();
		log_uartf(LOGDEBUG, "LM home position set.");
		model->pos = 0;
	} else if(cmd->type == lm_cmd_stop) {
		if(cmd->dir_hard) {
			dSPIN_Hard_Stop();
			log_uartf(LOGDEBUG, "LM Hard Stop Commit");
		} else {
			dSPIN_Soft_Stop();
			log_uartf(LOGDEBUG, "LM Soft Stop Commit");
		}
		model->state = lm_state_stop;
	} else if(cmd->type == lm_cmd_speed) {
		dSPIN_Run(cmd->dir_hard, cmd->pos_speed);
		log_uartf(LOGDEBUG, "LM speed: %ld, dir: %lu", cmd->pos_speed, (uint32_t)cmd->dir_hard);
		model->state = lm_state_speed;
		model->speed = cmd->pos_speed;
		model->dir = cmd->dir_hard;
	} else if(cmd->type == lm_cmd_pos) {
		if(L6470_BUSY1()) {
			dSPIN_Hard_Stop();
			log_uartf(LOGDEBUG, "Stop previous movement.");
		}
		dSPIN_Go_To(cmd->pos_speed);
		log_uartf(LOGDEBUG, "LM pos: %ld", cmd->pos_speed);
		model->state = lm_state_pos;
		model->speed = 0;
		model->pos = cmd->pos_speed;
	}
	return 1;
}
