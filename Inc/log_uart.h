#ifndef __LOG_UART_H__
#define __LOG_UART_H__
#include "stm32f1xx.h"

enum log_level {
	LOGDEBUG,
	LOGINFO,
	LOGWARN,
	LOGERROR,
};

void log_set_port(UART_HandleTypeDef *huart);
void log_set_level(enum log_level level);
int log_uart(enum log_level level, char* cstr);
int log_uart_raw(uint8_t* data, uint16_t len);
int log_uart_f(enum log_level level, const char* format, ...);

#endif
