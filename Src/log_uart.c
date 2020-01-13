#include "log_uart.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

static const char* LOG_DEBUG_HEAD = "[Debug] ";
static const char* LOG_INFO_HEAD = "[Info] ";
static const char* LOG_WARN_HEAD = "[Warn] ";
static const char* LOG_ERROR_HEAD = "[Error] ";
static enum log_level log_output_level = LOGDEBUG;
static char logbuf[1024];
static char timebuf[100];
static UART_HandleTypeDef *log_huart;

static int imax(int a, int b) {
	return a > b ? a : b;
}

void log_set_port(UART_HandleTypeDef *uart) {
	log_huart = uart;
}

void set_log_level(enum log_level level) {
	log_output_level = level;
}

int log_uart_raw(uint8_t* data, uint16_t len) {
	// 115200 Baud Rate ~= 80000 bps = 10KBps = 10 Byte / ms
	if (log_huart)
		return HAL_UART_Transmit(log_huart, data, len, imax(len / 8, 5));
	else
		return 1;
}

/*
 * Return 0 for ok, 1 to 3 for HAL_xxx, -1 for log level ignored.
 * */
int log_uart(enum log_level level, char* cstr) {
	const char* head = "";
	HAL_StatusTypeDef state;
	size_t slen;
	if (level >= log_output_level) {
		if (level == LOGDEBUG)
			head = LOG_DEBUG_HEAD;
		else if (level == LOGINFO)
			head = LOG_INFO_HEAD;
		else if (level == LOGWARN)
			head = LOG_WARN_HEAD;
		else if (level == LOGERROR)
			head = LOG_ERROR_HEAD;
		state = log_uart_raw((uint8_t*) head, strlen(head));
		if (state != HAL_OK)
			return state;
		slen = snprintf(timebuf, 100, "[%lu] ", HAL_GetTick());
		state = log_uart_raw((uint8_t*) timebuf, slen);
		if (state != HAL_OK)
			return state;
		slen = strlen(cstr);
		state = log_uart_raw((uint8_t*) cstr, slen);
		if (state != HAL_OK)
			return state;
		state = log_uart_raw((uint8_t*) "\r\n", 1);
		return state;
	} else {
		return -1;
	}
}

// output limit is 1023 char.
int log_uart_f(enum log_level level, const char* format, ...) {
	va_list args;
	va_start(args, format);
	vsnprintf(logbuf, 1024, format, args);
	int res = log_uart(level, logbuf);
	va_end(args);
	return res;
}
