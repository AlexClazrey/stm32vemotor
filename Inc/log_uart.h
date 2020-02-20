#ifndef __LOG_UART_H__
#define __LOG_UART_H__
#include "stm32f1xx.h"

enum logu_level {
	LOGU_TRACE,
	LOGU_DEBUG,
	LOGU_INFO,
	LOGU_WARN,
	LOGU_ERROR,
	LOGU_PANIC,
	LOGU_NONE,
};

enum logu_method {
	LOGU_DIRECT,
	LOGU_DMA,
};

int logu_init(UART_HandleTypeDef *uart, enum logu_method method);
void logu_setport(UART_HandleTypeDef *huart);
UART_HandleTypeDef *logu_getport();
void logu_setlevel(enum logu_level level);
enum logu_level logu_getlevel();
void logu_dma_txcplt_callback();
int logu_raw(char *data, uint16_t len);
int logu_s(enum logu_level level, const char *cstr);
int logu_f(enum logu_level level, const char* format, ...);

#endif
