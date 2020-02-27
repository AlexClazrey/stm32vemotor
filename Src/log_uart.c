#include "log_uart.h"
#include "util.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/*
 * Module Description
 * This can print log messages through a USART connection.
 * It has two ways of transmitting, using direct transmit which may take a lot of CPU time
 * or using DMA method, both methods depend on HAL library.
 * The default method is direct transmit. You can change it by logu_init().
 * You need to set up Pins and/or DMA channel first. You may use STM32 CubeMX.
 * Default log level is LOGU_DEBUG. If you want to log trace, you need to call logu_setlevel(LOGU_TRACE);
 * All symbols exported have "logu_" prefix.
 *
 * Init:
 * Init function logu_init() has two parameters, UART Port, enum logu_method.
 * If you use DMA transmit, logu_dma_txcplt_callback() must be called in HAL_UART_TxCpltCallback().
 * This is because multiple log requests may be fired when sending a log message via USART which is a slow process,
 * once previous sending is complete, this callback function will start to send the next message in queue.
 *
 * Macros:
 * macro LOG_HEAD_BUF_SIZE controls the max length of message head like "[Debug][10431] ".
 * macro LOG_LINE_BUF_SIZE controls the max length of one message.
 * macro LOG_TOTAL_BUF_SIZE is used in DMA mode, controlling total size of all messages in queue.
 * macro LOG_LINES_COUNT is used in DMA mode, which is the max number of messages in queue.
 * macro LOG_BAUDRATE is needed in direct mode to calculate timeout milliseconds.
 *
 * Main functions:
 * logu_f(<enum logu_level>, <format-string>, ...)
 * logu_s(<enum logu_level>, <cstr>)
 * logu_raw(<unsigned char*>, <len>)
 *
 * This module is reentrant with lock error result code -4 but not thread safe.
 *
 * */

/* LOG_LINE_BUF_SIZE must be no less than 30. */

/* ---- Settings ---- */
#define LOG_HEAD_BUF_SIZE 30
#define LOG_LINE_BUF_SIZE 200
#define LOG_TOTAL_BUF_SIZE 2000
#define LOG_LINES_COUNT 40
#define LOG_BAUDRATE 115200

// If you want to debug this module.
#define LOG_UART_DEBUG

static const char *LOG_TRACE_HEAD = "[Trace]";
static const char *LOG_DEBUG_HEAD = "[Debug]";
static const char *LOG_INFO_HEAD = "[Info] ";
static const char *LOG_WARN_HEAD = "[Warn] ";
static const char *LOG_ERROR_HEAD = "[Error]";
static const char *LOG_PANIC_HEAD = "[!!!PANIC!!!]";
static const char *LOG_LINEEND = "\r\n";

/* ---- Types ---- */
enum log_stage {
	log_stage_free, log_stage_issued, log_stage_half_issued, log_stage_filled, log_stage_fill_failed, log_stage_occupied,
};

struct log_meta {
	uint32_t end;
	uint32_t len;
	enum log_stage stage;
};

/* ---- Variables ---- */
static enum logu_level log_output_level = LOGU_DEBUG;
static enum logu_method log_output_method = LOGU_DIRECT;

// presume 12 bit is consumed to transmit one byte on average.
static int log_timeout_factor = LOG_BAUDRATE / 12000;
static int log_lineendlen = 0;

static char logbuf[LOG_TOTAL_BUF_SIZE] = { 0 };

static volatile int loglock = 0;
static struct log_meta logmetas[LOG_LINES_COUNT] = { 0 };
static volatile uint32_t logmetastart = 0;
static volatile uint32_t logmetatoissue = 0;
static volatile uint32_t logmetaend = 0;
#define LOGMETASIZE (sizeof(struct log_meta))

// peripherals
static UART_HandleTypeDef *log_huart;

/* ---- Functions ---- */

/* -- Utility -- */
static int imax(int a, int b) {
	return a > b ? a : b;
}

/* -- Setting functions -- */
// return 0 for good, 1 for bad.
int logu_init(UART_HandleTypeDef *uart, enum logu_method method) {
	logu_setport(uart);
	log_output_method = method;
	log_lineendlen = strlen(LOG_LINEEND);
	return 0;
}

void logu_setport(UART_HandleTypeDef *uart) {
	log_huart = uart;
}

UART_HandleTypeDef* logu_getport() {
	return log_huart;
}

void logu_setlevel(enum logu_level level) {
	log_output_level = level;
}

enum logu_level logu_getlevel() {
	return log_output_level;
}

/* -- Debug Trace functions -- */
#ifdef LOG_UART_DEBUG
struct logu_trace {
	const char* funcname;
	uint32_t time;
	uint32_t start, toissue, end;
};
static struct logu_trace ltrace[200] = {0};
static uint32_t ltracecur = 0;
static void ltracepush(const char* funcname) {
	struct logu_trace *current = ltrace + (ltracecur++);
	if(ltracecur == 200)
		ltracecur = 0;
	current->funcname = funcname;
	current->time = HAL_GetTick();
	current->start = logmetastart;
	current->toissue = logmetatoissue;
	current->end = logmetaend;
}
static int logmetaisvalid_it(void* metaitem, size_t index);
// return 0 for invalid, 1 for valid.
static int logmetaisvalid() {
	int isbet = cycbetween(logmetastart, logmetatoissue, logmetaend, LOG_LINES_COUNT);
	// this will return (index + 1) for the failed item
	uint32_t item = cycarriter(logmetas, LOGMETASIZE, logmetastart, logmetaend, LOG_LINES_COUNT, logmetaisvalid_it);
	if(!isbet || item) {
		return 0; // break here to debug
	} else
		return 1;
}

// line fill failed or disorder will trigger return
// return index + 1
static int logmetaisvalid_it(void* metaitem, size_t index) {
	// when index==start, previous one must be log_stage_free
	struct log_meta *item = (struct log_meta*)metaitem;
	enum log_stage previous = index > 0 ? (item-1)->stage : (item+LOG_LINES_COUNT-1)->stage;
	enum log_stage now = item->stage;
	if(now == log_stage_fill_failed) {
		return index + 1;
	}
	if(previous > now)
		return index + 1;
	else
		return 0;
}
static int logmetaisvalid_special(const char * caller_name) {
	int a = logmetaisvalid();
	if(a) {
		ltracepush(caller_name);
	} else {
		__NOP();
	}
	return a;
}
#define metavalid() logmetaisvalid_special(__func__)
#else
#define metavalid() 1
#endif

/* -- Transmit functions -- */
static int log_occupyline(uint32_t length, struct log_meta **out_item, uint32_t *out_from);
static int log_dma_issue();
static int log_tosend(uint32_t *out_startid, uint32_t *out_endid);
static void log_freesent();
/*
 * Return 0 to 3 for HAL_StatusTypeDef result from HAL UART functions,
 * Return 10 for data put in queue successfully.
 * Return 11 for sent by other reentrants which is mostly not an error.
 * Return -3 for too small total buffer.
 * Return -4 for lock problem.
 * Return -5 for init function is not properly called.
 * Return -6 for unknown reasons error.
 * Return -7 for LOG_LINES_COUNT too small.
 *
 */
int logu_raw(char *data, uint16_t len) {
	if (loglock) {
		return -4;
	}
	if (len == 0) {
		return HAL_OK;
	}
	if (log_huart) {
		if (log_output_method == LOGU_DMA) {
			uint32_t from = 0;
			struct log_meta *entry;
			if (!metavalid()) {
				return -6;
			}
			loglock = 1;
			int res = log_occupyline(len, &entry, &from);
			loglock = 0;
			if (!metavalid()) {
				return -6;
			}

			if (res < 0) {
				return res;
			}

			int cpyres = arrtocycarr(logbuf, from, entry->end, LOG_TOTAL_BUF_SIZE, data, len);

			if (cpyres < 0) {
				entry->stage = log_stage_fill_failed;
				return -6;
			}
			entry->stage = log_stage_filled;
			int issue = log_dma_issue();
			if (issue >= 0) {
				return issue;
			} else if (issue == -1) {
				return 11;
			} else if (issue == -2) {
				return 10;
			} else if (issue == -4) {
				return 10; // 这个时候肯定有一个成员在lock里面，只要它能跑完发送就没有问题。
			} else {
				return -6;
			}
		} else {
			loglock = 1;
			HAL_StatusTypeDef res = HAL_UART_Transmit(log_huart, (uint8_t*) data, len,
					imax(len / log_timeout_factor, 5));
			loglock = 0;
			return res;
		}
	} else
		return -5;
}

// in DMA TX Normal mode, this will be called and hal_uart state becomes ready after TC=1
void logu_dma_txcplt_callback() {
	log_freesent();
	log_dma_issue();
}

/*
 * return -1 for nothing to send.
 * return -2 for TX busy, which I suppose it is already occupied by DMA.
 * return -4 for lock error
 * return -6 for internal error.
 **/
static int log_dma_issue() {
	if (loglock) {
		return -4;
	}
	// to ensure logissued and logend is not modified.
	loglock = 1;
	if (logmetaend == logmetatoissue) {
		loglock = 0;
		return -1;
	}
	// TC 不一定表示它不处于Busy状态。
	if (__HAL_UART_GET_FLAG(log_huart, USART_SR_TC)
			&& log_huart->gState == HAL_UART_STATE_READY) {
		uint32_t start = 0, end = 0;
		int res = log_tosend(&start, &end);
		if (res < 0) {
			loglock = 0;
			return -6;
		}
		res = HAL_UART_Transmit_DMA(log_huart, (uint8_t*) logbuf + start, end - start);
		loglock = 0;
		return res;
	} else {
		loglock = 0;
		return -2;
	}
}

// TODO 你说如果发现高占用率的Warn或者内部状态检查不一致的错误应该用什么方式通知呢？
// TODO 如果在TC产生，然后freelines之前，有一个高等级中断又触发了一个DMA issue怎么办。
// TODO 这一点就需要把 toissue 改成一个FIFO队列。
// logmetas array must be initialized to zero.
// verb occupy/issue needs to be sequential, so we need lock to protect it.
// verb finish is sequential because you cannot have reentrants on the same interrupt
// verb fill is not sequential if reentrant
// 除了占用，对 LOG_LINES_COUNT 的 addu 操作只能往回减，可以保证首尾相连，不能往后加

// returns the index of the next char of last finished char
static uint32_t log_lastfinished() {
	return logmetas[addu(logmetastart, -1, LOG_LINES_COUNT)].end;
}
// returns the index of the next char of last issued char.
// doesn't work on half issued items, caller needs to take care of that.
static uint32_t log_lastissued() {
	return logmetas[addu(logmetatoissue, -1, LOG_LINES_COUNT)].end;
}
// returns index of the next char of last occupied char.
// needs to work even the array is empty.
static uint32_t log_lastoccupied() {
	return logmetas[addu(logmetaend, -1, LOG_LINES_COUNT)].end;
}
static uint32_t log_metaisempty() {
	return logmetastart == logmetaend;
}
static uint32_t log_availlines() {
	return addu(diffu(logmetaend, logmetastart, LOG_LINES_COUNT), -1, LOG_LINES_COUNT);
}
static uint32_t log_availchars() {
	if (log_metaisempty()) {
		return LOG_TOTAL_BUF_SIZE;
	}
	return addu(diffu(log_lastoccupied(), log_lastfinished(), LOG_TOTAL_BUF_SIZE), -1, LOG_TOTAL_BUF_SIZE);
}
// return 0 for good, -3 for too small total buffer, -7 for too less lines.
// this will change logmetaend if returns 0.
static int log_occupyline(uint32_t length, struct log_meta **out_item, uint32_t *out_from) {
	if (log_availlines() == 0) {
		return -7;
	}
	if (log_availchars() < length) {
		return -3;
	}
	*out_item = logmetas + logmetaend;
	*out_from = log_lastoccupied();
	logmetaend = addu(logmetaend, 1, LOG_LINES_COUNT);
	(*out_item)->end = addu(*out_from, length, LOG_TOTAL_BUF_SIZE);
	(*out_item)->len = length;
	(*out_item)->stage = log_stage_occupied;
	return 0;
}

static void log_freelines(uint32_t start, uint32_t end);
static int log_freelines_it(void *metaitem, size_t index);

static void log_freesent() {
	log_freelines(logmetastart, logmetatoissue);
	if (!metavalid()) {
		__NOP();
	}
}
// mark logmetas[start] to logmetas[end-1] as free space.
// this will change logmetastart
// start is included, end is not included.
static void log_freelines(uint32_t start, uint32_t end) {
	cycarriter(logmetas, LOGMETASIZE, start, end, LOG_LINES_COUNT, log_freelines_it);
	logmetastart = end;
}
static int log_freelines_it(void *metaitem, size_t index) {
	((struct log_meta*) metaitem)->stage = log_stage_free;
	return 0;
}

static int log_set_issued_it(void *metaitem, size_t index);
static uint32_t log_lastfilled_nowrap();
static int log_lastfilled_nowrap_it(void *metaitem, size_t index);
static int log_lineiswrapped(struct log_meta *item) {
	return item->end < item->len;
}

// this will change logmetatoissue
// caller should ensure there is at least one line to send.
// return -6 for internal error
static int log_tosend(uint32_t *out_startid, uint32_t *out_endid) {
	if (!metavalid()) {
		return -6;
	}
	while (logmetas[logmetatoissue].stage == log_stage_fill_failed) {
		logmetas[logmetatoissue].stage = log_stage_issued;
		logmetatoissue = addu(logmetatoissue, 1, LOG_LINES_COUNT);
	}
	if (logmetatoissue == logmetaend) {
		return -1;
	}
	if (logmetas[logmetatoissue].stage == log_stage_half_issued) {
		*out_startid = 0;
		logmetas[logmetatoissue].stage = log_stage_issued;
		logmetatoissue = addu(logmetatoissue, 1, LOG_LINES_COUNT);
		// 只会有一个half issued发生，跳过这个就好了
	} else {
		*out_startid = log_lastissued();
	}
	if (logmetatoissue == logmetaend) {
		*out_endid = log_lastissued();
	} else {
		uint32_t to_item = log_lastfilled_nowrap();
		if (to_item < 0)
			return to_item;
		cycarriter(logmetas, LOGMETASIZE, logmetatoissue, to_item, LOG_LINES_COUNT, log_set_issued_it);
		struct log_meta *last = logmetas + to_item;
		if (log_lineiswrapped(last)) {
			logmetatoissue = to_item;
			last->stage = log_stage_half_issued;
			*out_endid = LOG_TOTAL_BUF_SIZE;
		} else {
			logmetatoissue = addu(to_item, 1, LOG_LINES_COUNT);
			last->stage = log_stage_issued;
			*out_endid = last->end;
		}
	}
	if (!metavalid()) {
		return -6;
	}
	return 0;
}

static int log_set_issued_it(void *metaitem, size_t index) {
	((struct log_meta*) metaitem)->stage = log_stage_issued;
	return 0;
}

// return index of last filled log_meta item without circular wrapping
// return -1 if nothing to send
static uint32_t log_lastfilled_nowrap() {
	int notfilled = cycarriter(logmetas, LOGMETASIZE, logmetatoissue, logmetaend,
	LOG_LINES_COUNT, log_lastfilled_nowrap_it) - 1;
	// notfilled 0 to LOG_LINES_COUNT-1 的时候是指向具体的项目。
	if (notfilled == -1) {
		// 有两种情况，一种是全部filled，一种是没有东西可发，一种是half issued，后两种是调用者负责
		return addu(logmetaend, -1, LOG_LINES_COUNT);
	}
	return notfilled;
}
static int log_lastfilled_nowrap_it(void *metaitem, size_t index) {
	struct log_meta *item = (struct log_meta*) metaitem;
	if (item->stage != log_stage_filled) {
		return addu(index, -1, LOG_LINES_COUNT) + 1; // return non-zero value will break iteration
	}
	if (item->end < item->len) {
		return index + 1;
	}
	return 0;
}

/* -- Content Functions -- */
static int log_fill(char *dest, enum logu_level level, const char *content);
static int log_head(char *dest, enum logu_level level);
static int log_tail(char *dest);

/*
 * Return 0 to 3 for HAL_StatusTypeDef result from HAL UART functions,
 * Return 10 for data put in queue successfully.
 * Return 11 for sent by other entrants which is mostly not an error.
 * Return -1 for ignored due to log level setting.
 * Return -2 for too small line buffer.
 * Return -3 for too small total buffer.
 * Return -4 for lock problem.
 * Return -5 for init function is not properly called.
 * Return -6 for unknown reasons error.
 * Return -7 for LOG_LINES_COUNT too small.
 *
 */
int logu_s(enum logu_level level, const char *cstr) {
	char linebuf[LOG_LINE_BUF_SIZE];
	int slen = log_fill(linebuf, level, cstr);
	if (slen < 0)
		return slen;
	else
		return logu_raw(linebuf, slen);
}

/*
 * Return 0 to 3 for HAL_StatusTypeDef result from HAL UART functions,
 * Return 10 for data put in queue successfully.
 * Return 11 for sent by other entrants which is mostly not an error.
 * Return -1 for ignored due to log level setting.
 * Return -2 for too small line buffer.
 * Return -3 for too small total buffer.
 * Return -4 for lock problem.
 * Return -5 for init function is not properly called.
 * Return -6 for unknown reasons error.
 * Return -7 for LOG_LINES_COUNT too small.
 *
 */
int logu_f(enum logu_level level, const char *format, ...) {
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
			return logu_raw(linebuf, clen);
	} else {
		return -2;
	}
}

// eliminate duplicate code at an effort of extra memory of LOG_HEAD_BUF_SIZE
// dest must has size of LOG_LINE_BUF_SIZE.
// dest can overlap with content
static int log_fill(char *dest, enum logu_level level, const char *content) {
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

static int log_head(char *dest, enum logu_level level) {
	const char *head = ""; // init this value if level can't find a match
	if (level >= log_output_level) {
		// prepare header
		if (level == LOGU_TRACE)
			head = LOG_TRACE_HEAD;
		else if (level == LOGU_DEBUG)
			head = LOG_DEBUG_HEAD;
		else if (level == LOGU_INFO)
			head = LOG_INFO_HEAD;
		else if (level == LOGU_WARN)
			head = LOG_WARN_HEAD;
		else if (level == LOGU_ERROR)
			head = LOG_ERROR_HEAD;
		else if (level == LOGU_PANIC)
			head = LOG_PANIC_HEAD;
		int slen = strlen(head);
		if (slen >= LOG_HEAD_BUF_SIZE) {
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
