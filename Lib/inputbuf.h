#ifndef __INPUTBUF_H__
#define __INPUTBUF_H__

#include "stm32f1xx.h"

/* Configurations start */
// 用户可以把全局设置单独放在一个文件里
#include "config.h"

// 这个模块提供一个便于操作的DMA input buffer。收集输入的数据之后可以按照行读出，也可以按照二进制原始数据读出。
// 接入方法：
// 在开头调用init函数，init函数传入新的struct和参数。
// 然后在 input idle 中断里面调用 inputbuf_idlehandler()，
// 在 dma error 中断里面调用 inputbuf_error_handler()，
// 在 dma halfcplt 里面调用 inputbuf_rxhalfcplt_callback()，
// 在 dma cplt 里面调用 inputbuf_rxcplt_callback()。
// 使用方法：
// 在主循环里面定期调用 inputbuf_read_toline() 或者 inputbuf_read_raw() 。
// 如果 read_toline() 的 bool* 参数变成了 true，代表读取到了行尾或者行缓冲已满。
// 这个时候使用 inputbuf_getline() 可以读取从上一个行尾到这一个行尾的数据，
// 如果在下一次 read_toline() 之前没有使用 inputbuf_getline() 这个函数，那么这行数据将被丢弃。

// inputbuf 可以用栈内存或者堆内存，通过 INPUTBUF_MEM_MODE 这个 macro 来设置，
// 在栈内存模式下所有Inputbuf示例使用的缓冲区大小相通，
// 在堆内存模式下可以不同，内存在 init_heap 函数里分配。
#define INPUTBUF_STACK 1
#define INPUTBUF_HEAP 2

// 默认使用栈内存模式
#ifndef INPUTBUF_MEM_MODE
#define INPUTBUF_MEM_MODE INPUTBUF_STACK
#endif

#define INPUTBUF_MEM_STACK (INPUTBUF_MEM_MODE == INPUTBUF_STACK)
#define INPUTBUF_MEM_HEAP (INPUTBUF_MEM_MODE == INPUTBUF_HEAP)

#if INPUTBUF_MEM_STACK
// 当它是栈内存模式时，在这里设置使用的参数
#ifndef INPUTBUF_BUFSIZE 
#define INPUTBUF_BUFSIZE  100
#endif
#ifndef INPUTBUF_DMA_RANGE
#define INPUTBUF_DMA_RANGE INPUTBUF_BUFSIZE
#endif
#ifndef INPUTBUF_LINESIZE
#define INPUTBUF_LINESIZE 70
#endif

#elif INPUTBUF_MEM_HEAP
// 当它是堆内存模式时，参数在init函数里面给出，这里不用改动。
#define INPUTBUF_BUFSIZE (ibuf->bufsize)
#define INPUTBUF_DMA_RANGE (ibuf->bufsize - 1)
#define INPUTBUF_LINESIZE (ibuf->linesize)

#else
// 如果都不是那么报错
#error INPUTBUF_MEM_MODE can be INPUTBUF_STACK or INPUTBUF_HEAP.

#endif 

/* Other configurations are in inputbuf.c */
/* Configurations end */

#define linelenlimit (INPUTBUF_LINESIZE-1)

struct inputbuf {
    UART_HandleTypeDef *huart;
    uint32_t start;
    volatile uint32_t end;
    uint32_t linelen;
    _Bool lineendmet;
    const char* waitforstr;
#if INPUTBUF_MEM_STACK
    char buf[INPUTBUF_BUFSIZE + 1];
    char line[INPUTBUF_LINESIZE + 1];
#elif INPUTBUF_MEM_HEAP
    char *buf;
    uint32_t bufsize;
    char *line;
    uint32_t linesize;
#else
#error INPUTBUF_MEM_MODE can be INPUTBUF_STACK or INPUTBUF_HEAP.
#endif
};

char* inputbuf_get(struct inputbuf *ibuf);
#if INPUTBUF_MEM_STACK
void inputbuf_init_stack(struct inputbuf *ibuf, UART_HandleTypeDef *huart);
#else 
void inputbuf_init_heap(struct inputbuf *ibuf, UART_HandleTypeDef *huart, uint32_t bufsize, uint32_t linesize);
#endif
void inputbuf_start(struct inputbuf *ibuf);
void inputbuf_stop(struct inputbuf *ibuf);

void inputbuf_enableidleinterrupt(struct inputbuf* ibuf);

// peripheral
UART_HandleTypeDef *inputbuf_getport(struct inputbuf *ibuf);
void inputbuf_idleinterrupt(struct inputbuf *ibuf);
void inputbuf_error_handler(struct inputbuf *ibuf);
void inputbuf_rxhalfcplt_callback(struct inputbuf *ibuf);
void inputbuf_rxcplt_callback(struct inputbuf *ibuf);

// mask input until keyword
void inputbuf_waitfor(struct inputbuf *ibuf, const char* keyword);
void inputbuf_cancelwait(struct inputbuf *ibuf);
_Bool inputbuf_iswaiting(struct inputbuf *ibuf);

// read line-end trigger mode
char* inputbuf_read_toline(struct inputbuf *ibuf, _Bool *lineend);
char* inputbuf_getline(struct inputbuf *ibuf);
void inputbuf_clearline(struct inputbuf *ibuf);

// read raw mode
uint32_t inputbuf_read_raw(struct inputbuf *ibuf, char* dest, uint32_t targetcount, uint32_t readcount);

#endif
