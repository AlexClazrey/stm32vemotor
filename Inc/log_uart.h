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

void log_init(UART_HandleTypeDef *uart, enum log_method method);
void log_setport(UART_HandleTypeDef *huart);
void log_setlevel(enum log_level level);
int log_uart(enum log_level level, char* cstr);
HAL_StatusTypeDef log_uartraw(uint8_t* data, uint16_t len);
int log_uartf(enum log_level level, const char* format, ...);

#endif
