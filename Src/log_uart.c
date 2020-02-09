#include "log_uart.h"
#include "util.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/*
 * Module Description
 * This can print log messages through a USART connection.
 * It has two ways of transmitting, using direct transmit which may take a lot CPU time
 * or using DMA method, both methods depend on HAL library.
 * The default method is direct transmit. You can change it by log_init().
 * You need to set up Pins and/or DMA channel first. You may use STM32 CubeMX.
 *
 * Init function log_init() has two parameters, UART Port, Enum log_method.
 * If you use DMA transmit, log_dma_txcplt_callback() must be called in HAL_UART_TxCpltCallback().
 * Because multiple log requests may be fired when sending a log message via USART which is a slow process,
 * once previous sending is complete, this callback function will start to send the next message in queue.
 *
 * macro LOG_HEAD_BUF_SIZE control buffer size of message head like "[Debug][10431] ".
 * macro LOG_LINE_BUF_SIZE controls buffer size of one message.
 * macro LOG_TOTAL_BUF_SIZE is used in DMA mode, controlling total size of all messages in queue.
 * macro LOG_BAUDRATE is needed in direct mode to calculate timeout milliseconds.
 *
 * Main functions:
 * log_uartf(<enum log_level>, <format-string>, ...)
 * log_uart(<enum log_level>, <cstr>)
 * log_uartraw(<unsigned char*>, <len>)
 *
 * This module is reentrant with lock error result code -4 but not thread safe.
 *
 * */

/* LOG_LINE_BUF_SIZE must be no less than 25. */
#define LOG_HEAD_BUF_SIZE 25
#define LOG_LINE_BUF_SIZE 200
#define LOG_TOTAL_BUF_SIZE 1000
#define LOG_BAUDRATE 115200

static const char *LOG_DEBUG_HEAD = "[Debug]";
static const char *LOG_INFO_HEAD = "[Info] ";
static const char *LOG_WARN_HEAD = "[Warn] ";
static const char *LOG_ERROR_HEAD = "[Error]";
static const char *LOG_LINEEND = "\r\n";

static enum log_level log_output_level = LOGDEBUG;
static enum log_method log_output_method = LOGDIRECT;

// presume 12 bit is consumed to transmit one byte on average.
static int log_timeout_factor = LOG_BAUDRATE / 12000;
static int log_lineendlen = 0;

static char logbuf[LOG_TOTAL_BUF_SIZE] = { 0 };
static volatile uint32_t logstart = 0;
static volatile uint32_t logissued = 0;
static volatile uint32_t logend = 0;
// log buffer needs a lock in reentrant situations
// this ensures all RW to logstart/logend/logissued is in sync.
static volatile int loglock = 0;

// peripherals
static UART_HandleTypeDef *log_huart;

static int imax(int a, int b) {
	return a > b ? a : b;
}

// return 0 for good, 1 for bad.
int log_init(UART_HandleTypeDef *uart, enum log_method method) {
	log_setport(uart);
	log_output_method = method;
	log_lineendlen = strlen(LOG_LINEEND);
	return 0;
}

void log_setport(UART_HandleTypeDef *uart) {
	log_huart = uart;
}

void log_setlevel(enum log_level level) {
	log_output_level = level;
}


static int log_dma_issue();
/*
 * Return 0 to 3 for HAL_StatusTypeDef result from HAL UART functions,
 * Return 10 for data put in queue successfully.
 * Return 11 for sent by other entrants which is mostly not an error.
 * Return -3 for too small total buffer.
 * Return -4 for lock problem.
 * Return -5 for init function is not properly called.
 * Return -6 for unknown reasons error.
 *
 */
int log_uartraw(char *data, uint16_t len) {
	if (loglock) {
		return -4;
	}
	if (len == 0) {
		return HAL_OK;
	}
	if (log_huart) {
		if (log_output_method == LOGDMA) {
			loglock = 1;
			// Read Write logstart/logend should be in lock region.
			if (diffu(logend, logstart - 1, LOG_TOTAL_BUF_SIZE) < len) {
				loglock = 0;
				return -3;
			}
			int oldend = logend;
			int newend = logend = addu(logend, len, LOG_TOTAL_BUF_SIZE);
			loglock = 0;

			int cpyres = arrtocycarr(logbuf, oldend, newend, LOG_TOTAL_BUF_SIZE, data, len);
			// 这里发送前不作是不是已经填充内容的检验了不然会很复杂。
			// 假设Copy的速度总是比发送的速度快，只要不要在Copy的过程里面中断太长时间就可以。
			// 原理是因为如果在没有Copy完的地方有中断，中断里面先请求了DMA指令，
			// 只要那个中断里面没有断点卡时间，那么发送UART的速度并不快，那个中断回来之后这里可以很快复制好。
			// 从而这一段耗时的部分不需要放在lock里面。

			if (cpyres < 0) {
				return -6;
			}
			int issue = log_dma_issue();
			if (issue >= 0) {
				return issue;
			} else if (issue == -1) {
				return 11;
			} else if (issue == -2) {
				return 10;
			} else if (issue == -4) {
				return -4;
			} else {
				return -6;
			}
		} else {
			loglock = 1;
			HAL_StatusTypeDef res = HAL_UART_Transmit(log_huart, (uint8_t*)data, len, imax(len / log_timeout_factor, 5));
			loglock = 0;
			return res;
		}
	} else
		return -5;
}

void log_dma_txcplt_callback() {
	// TODO 这里可能有隐藏bug，在这句话之前，TC清除之后如果又有东西触发了一次issue的话，那么这个issue就不是之前的issue值了。
	// 考虑到这种情况不可能层叠很多次，就算这个函数被多次重入，用一个长度是5的队列也够。
	logstart = logissued;
	log_dma_issue();
}

/*
 * return -1 for nothing to send.
 * return -2 for TX busy, which I suppose it is already occupied by DMA.
 * return -4 for lock error
 **/
static int log_dma_issue() {
	HAL_StatusTypeDef res;
	if (loglock) {
		return -4;
	}
	// to ensure logissued and logend is not modified.
	loglock = 1;
	if (logend == logissued) {
		loglock = 0;
		return -1;
	}
	if (__HAL_UART_GET_FLAG(log_huart, USART_SR_TC)) {
		if (logend > logissued) {
			// if circular array is like a normal array
			// log from start to end
			res = HAL_UART_Transmit_DMA(log_huart, (uint8_t*)logbuf + logissued, logend - logissued);
			logissued = logend;
		} else {
			// if circular array end cursor is before start cursor
			// log from start to buffer end.
			res = HAL_UART_Transmit_DMA(log_huart, (uint8_t*)logbuf + logissued, LOG_TOTAL_BUF_SIZE - logissued);
			logissued = 0;
		}
		loglock = 0;
		return res;
	} else {
		loglock = 0;
		return -2;
	}
}

static int log_fill(char *dest, enum log_level level, const char *content);
static int log_head(char *dest, enum log_level level);
static int log_tail(char *dest);

/*
 * Return 0 to 3 for HAL_StatusTypeDef result from HAL UART functions,
 * Return -1 for ignored due to log level setting.
 * Return -2 for too small line buffer.
 * Return -3 for too small total buffer.
 * Return -4 for lock problem.
 * Return -5 for init function is not called.
 *
 */
int log_uart(enum log_level level, const char *cstr) {
	char linebuf[LOG_LINE_BUF_SIZE];
	int slen = log_fill(linebuf, level, cstr);
	if (slen < 0)
		return slen;
	else
		return log_uartraw(linebuf, slen);
}

/*
 * Return 0 to 3 for HAL_StatusTypeDef result from HAL UART functions,
 * Return -1 for ignored due to log level setting.
 * Return -2 for too small line buffer.
 * Return -3 for too small total buffer.
 * Return -4 for lock problem.
 * Return -5 for init function is not called.
 *
 */
int log_uartf(enum log_level level, const char *format, ...) {
	char linebuf[LOG_LINE_BUF_SIZE];
	va_list args;
	va_start(args, format);
	int clen = vsnprintf(linebuf, LOG_LINE_BUF_SIZE, format, args);
	va_end(args);
	if (clen > 0 && clen < LOG_LINE_BUF_SIZE) {
		clen = log_fill(linebuf, level, linebuf);
		if (clen < 0)
			return clen;
		else
			return log_uartraw(linebuf, clen);
	} else {
		return -2;
	}
}

// eliminate duplicate code at an effort of extra memory of LOG_HEAD_BUF_SIZE
// dest must has size of LOG_LINE_BUF_SIZE.
// dest can overlap with content
static int log_fill(char *dest, enum log_level level, const char *content) {
	char headbuf[LOG_HEAD_BUF_SIZE];
	int slen = log_head(headbuf, level);
	if (slen < 0) {
		return slen;
	}
	// may be unsigned comparison, so no subtraction should be placed
	if (slen + log_lineendlen > LOG_LINE_BUF_SIZE) {
		return -2;
	}
	size_t csize = LOG_LINE_BUF_SIZE - slen - log_lineendlen + 1; // must be positive
	size_t clen = strlen(content);
	if (clen > csize) {
		return -2;
	}
	memmove(dest + slen, content, clen);
	memcpy(dest, headbuf, slen);
	slen += clen;
	slen += log_tail(dest + slen);
	return slen;
}

static int log_head(char *dest, enum log_level level) {
	const char *head = ""; // init this value if level can't find a match
	if (level >= log_output_level) {
		// prepare header
		if (level == LOGDEBUG)
			head = LOG_DEBUG_HEAD;
		else if (level == LOGINFO)
			head = LOG_INFO_HEAD;
		else if (level == LOGWARN)
			head = LOG_WARN_HEAD;
		else if (level == LOGERROR)
			head = LOG_ERROR_HEAD;
		int slen = strlen(head);
		if(slen >= LOG_HEAD_BUF_SIZE) {
			return -2;
		}
		strcpy(dest, head);
		slen += snprintf(dest + slen, LOG_HEAD_BUF_SIZE - slen, "[%lu] ", HAL_GetTick()); // 2^32 is 10-digit, combined with "[]\0" is 13 chars.
		// no checking length here, it will output will a truncated time stamp if head buffer is too small.
		return slen;
	}
	return -1;
}

static int log_tail(char *dest) {
	memcpy(dest, LOG_LINEEND, log_lineendlen);
	return log_lineendlen;
}
