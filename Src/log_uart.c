#include "log_uart.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/*
 * Module Description
 * This can print log messages through a USART connection.
 * It has two ways of transmitting, using direct transmit which may take a lot CPU time
 * or using DMA method, both methods depend on HAL library.
 * The default method is direct transmit. You can change it by log_init().
 *
 * You need to set up Pins and/or DMA channel first. You may use STM32 CubeMX.
 *
 * Initialization function log_init() has two parameters, UART Port and Enum log_method.
 * You only need to call this function before logging to make it work.
 *
 * log_uartf() which accepts C printf() format string uses a buffer to store output,
 * this buffer size is controlled by macro LOG_BUF_SIZE.
 * If you use direct transmit, macro LOG_BAUDRATE is needed to calculate timeout milliseconds.
 *
 * Main functions:
 * log_uartf(<enum log_level>, <format-string>, ...)
 * log_uart(<enum log_level>, <cstr>)
 * log_uartraw(<unsigned char*>, <len>)
 *
 * */

#define LOG_BUF_SIZE 1024
#define LOG_BAUDRATE 115200

static const char* LOG_DEBUG_HEAD = "[Debug] ";
static const char* LOG_INFO_HEAD = "[Info] ";
static const char* LOG_WARN_HEAD = "[Warn] ";
static const char* LOG_ERROR_HEAD = "[Error] ";

static enum log_level log_output_level = LOGDEBUG;
static enum log_method log_output_method = LOGDIRECT;
// presume 14 bit is consumed to transmit one byte on average.
static int log_timeout_factor = LOG_BAUDRATE / 14000;

static char logbuf[LOG_BUF_SIZE];
static char timebuf[100];

static UART_HandleTypeDef *log_huart;

static int imax(int a, int b) {
	return a > b ? a : b;
}

void log_init(UART_HandleTypeDef *uart, enum log_method method) {
	log_setport(uart);
	log_output_method = method;
}

void log_setport(UART_HandleTypeDef *uart) {
	log_huart = uart;
}

void log_setlevel(enum log_level level) {
	log_output_level = level;
}

// return 0 for good, 1 for bad.
HAL_StatusTypeDef log_uartraw(uint8_t* data, uint16_t len) {
	if (log_huart) {
		if(log_output_method == LOGDMA)
			return HAL_UART_Transmit_DMA(log_huart, data, len);
		else
			return HAL_UART_Transmit(log_huart, data, len, imax(len / log_timeout_factor, 5));
	}
	else
		return HAL_ERROR;
}

/*
 * Return 0 for ok, 1 to 3 for HAL_xxx, -1 for ignored due to log level.
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
		state = log_uartraw((uint8_t*) head, strlen(head));
		if (state != HAL_OK)
			return state;
		slen = snprintf(timebuf, 100, "[%lu] ", HAL_GetTick());
		state = log_uartraw((uint8_t*) timebuf, slen);
		if (state != HAL_OK)
			return state;
		slen = strlen(cstr);
		state = log_uartraw((uint8_t*) cstr, slen);
		if (state != HAL_OK)
			return state;
		state = log_uartraw((uint8_t*) "\r\n", 2);
		return state;
	} else {
		return -1;
	}
}

// output limit is 1023 char.
int log_uartf(enum log_level level, const char* format, ...) {
	va_list args;
	va_start(args, format);
	vsnprintf(logbuf, 1024, format, args);
	int res = log_uart(level, logbuf);
	va_end(args);
	return res;
}
