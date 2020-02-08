#ifndef __LOG_UART_H__
#define __LOG_UART_H__
#include "stm32f1xx.h"

enum log_level {
	LOGDEBUG,
	LOGINFO,
	LOGWARN,
	LOGERROR,
};

enum log_method {
	LOGDIRECT,
	LOGDMA,
};

int log_init(UART_HandleTypeDef *uart, enum log_method method);
void log_setport(UART_HandleTypeDef *huart);
void log_setlevel(enum log_level level);
void log_dma_txcplt_callback();
int log_uartraw(char *data, uint16_t len);
int log_uart(enum log_level level, const char *cstr);
int log_uartf(enum log_level level, const char* format, ...);

#endif
