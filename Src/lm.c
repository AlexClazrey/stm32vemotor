#include "lm.h"
#include "./DSpin/dspin.h"
#include "log_uart.h"
#include <string.h>


/*
 * Return 0 for success, 1 for failed.
 * */
int lm_commit(struct lm_cmd *cmd, volatile struct lm_model *model) {
	if(cmd == NULL || model == NULL) {
		log_uart_f(LOGERROR, "LM lm_commit got null pointer.");
		return 1;
	}
	if(cmd->type == lm_cmd_empty) {
		return 0;
	} else if(cmd->type == lm_cmd_reset) {
		dSPIN_Reset_Device();
		log_uart_f(LOGDEBUG, "LM chip reset.");
		model->state = lm_state_reset;
		model->pos = 0;
		model->speed = 0;
		model->dir = 0;
	} else if(cmd->type == lm_cmd_set_home) {
		dSPIN_Reset_Pos();
		log_uart_f(LOGDEBUG, "LM home position set.");
		model->pos = 0;
	} else if(cmd->type == lm_cmd_stop) {
		if(cmd->dir_hard) {
			dSPIN_Hard_Stop();
			log_uart_f(LOGDEBUG, "LM Hard Stop Commit");
		} else {
			dSPIN_Soft_Stop();
			log_uart_f(LOGDEBUG, "LM Soft Stop Commit");
		}
		model->state = lm_state_stop;
	} else if(cmd->type == lm_cmd_speed) {
		dSPIN_Run(cmd->dir_hard, cmd->pos_speed);
		log_uart_f(LOGDEBUG, "LM speed: %ld, dir: %lu", cmd->pos_speed, (uint32_t)cmd->dir_hard);
		model->state = lm_state_speed;
		model->speed = cmd->pos_speed;
		model->dir = cmd->dir_hard;
	} else if(cmd->type == lm_cmd_pos) {
		if(L6470_BUSY1()) {
			dSPIN_Hard_Stop();
			log_uart_f(LOGDEBUG, "Stop previous movement.");
		}
		dSPIN_Go_To(cmd->pos_speed);
		log_uart_f(LOGDEBUG, "LM pos: %ld", cmd->pos_speed);
		model->state = lm_state_pos;
		model->speed = 0;
		model->pos = cmd->pos_speed;
	}
	memset(cmd, 0, sizeof(struct lm_cmd));
	cmd->type = lm_cmd_empty;
	return 0;
}
