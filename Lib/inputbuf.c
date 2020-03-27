#include "inputbuf.h"
#include <string.h>

#include "log_uart.h"

#if INPUTBUF_MEM_HEAP
#include <stdlib.h>
#endif


/*
 * Module Description:
 * This module can give you an input buffer to manage DMA inputs easily, such as UART DMA user input.
 *
 *
 * */


/* Configurations start */

const char inputbuf_lineends[] = {'\0', '\n', '\r'};
/* Configuartions end */


#define true 1
#define false 0

char* inputbuf_get(struct inputbuf *ibuf) {
    return ibuf->buf;
}
static void inputbuf_setend(struct inputbuf *ibuf, uint32_t end) {
    ibuf->end = end;
}


#if INPUTBUF_MEM_STACK
void inputbuf_init_stack(struct inputbuf *ibuf, UART_HandleTypeDef *huart) {
    memset(ibuf, 0, sizeof(struct inputbuf));
    ibuf->huart = huart;
}
#else
void inputbuf_init_heap(struct inputbuf *ibuf, UART_HandleTypeDef *huart, uint32_t bufsize, uint32_t linesize) {
    memset(ibuf, 0, sizeof(struct inputbuf));
    ibuf->huart = huart;
    ibuf->bufsize = bufsize + 1;
    ibuf->linesize = linesize + 1;
    ibuf->buf = (char*)calloc(INPUTBUF_BUFSIZE, sizeof(char));
    ibuf->line = (char*)calloc(INPUTBUF_LINESIZE, sizeof(char));
}
void inputbuf_free_heap(struct inputbuf* ibuf) {
    if(ibuf->buf)
        free(ibuf->buf);
    if(ibuf->line)
        free(ibuf->line);
}
#endif

void inputbuf_start(struct inputbuf *ibuf) {
    ibuf->huart->Instance->SR;
    ibuf->huart->Instance->DR;
    ibuf->start = 0;
    ibuf->end = 0;
    __HAL_UART_ENABLE_IT(ibuf->huart, UART_IT_IDLE);
    HAL_UART_Receive_DMA(ibuf->huart, (uint8_t *)inputbuf_get(ibuf), INPUTBUF_DMA_RANGE);
}

void inputbuf_stop(struct inputbuf *ibuf) {
    HAL_UART_DMAStop(ibuf->huart);

}

UART_HandleTypeDef *inputbuf_getport(struct inputbuf *ibuf) {
    return ibuf->huart;
}

void inputbuf_enableidleinterrupt(struct inputbuf* ibuf) {
    __HAL_UART_ENABLE_IT(ibuf->huart, UART_IT_IDLE);
}

void inputbuf_idleinterrupt(struct inputbuf *ibuf) {
    if (__HAL_UART_GET_FLAG(ibuf->huart, USART_SR_IDLE)) {
        __HAL_UART_CLEAR_FLAG(ibuf->huart, USART_SR_IDLE);
        HAL_UART_DMAPause(ibuf->huart);
        uint32_t dmacnt = __HAL_DMA_GET_COUNTER(ibuf->huart->hdmarx);
        uint32_t len = INPUTBUF_DMA_RANGE - dmacnt;
        inputbuf_setend(ibuf, len);
        HAL_UART_DMAResume(ibuf->huart);
    }
}

void inputbuf_rxhalfcplt_callback(struct inputbuf *ibuf) {
    inputbuf_setend(ibuf, INPUTBUF_DMA_RANGE / 2);    
}

void inputbuf_rxcplt_callback(struct inputbuf *ibuf) {
    inputbuf_setend(ibuf, 0);
}

void inputbuf_error_handler(struct inputbuf *ibuf) {
#if INPUTBUF_MEM_STACK
    inputbuf_init_stack(ibuf, ibuf->huart);
#else
    inputbuf_free_heap(ibuf);
    inputbuf_init_heap(ibuf, ibuf->huart, ibuf->bufsize - 1, ibuf->linesize - 1);
#endif
    inputbuf_stop(ibuf);
    inputbuf_start(ibuf);
}

// 我们简单一点逐个比较字符串的匹配，O(n^2)。
enum inputbuf_wait {
    strmatch,
    strpartmatch, // part match 表示已经收到的字符串末尾匹配目标字符串的开头。
    strnotmatch,
};

static enum inputbuf_wait inputbuf_match(struct inputbuf *ibuf) {
    if(ibuf->waitforstr == NULL)
        return strmatch;
    size_t bufcursor = ibuf->start, tarcursor = 0;
    while(bufcursor != ibuf->end && ibuf->waitforstr[tarcursor]) {
        if(ibuf->buf[bufcursor] != ibuf->waitforstr[tarcursor]) {
            return strnotmatch;
        }
        tarcursor++;
        bufcursor++;
        if(bufcursor == INPUTBUF_DMA_RANGE) {
            bufcursor = 0;
        }
    }
    if(ibuf->waitforstr[tarcursor]) {
        return strpartmatch;
    } else {
        return strmatch;
    }
}

// return true for proceed, return false for pass.
static _Bool inputbuf_waitcheck(struct inputbuf *ibuf) {
    if(ibuf->waitforstr == NULL)
        return true;
    while(ibuf->start != ibuf->end) {
        // wait actions 
        enum inputbuf_wait waitres = inputbuf_match(ibuf);
        if(waitres == strmatch) {
            inputbuf_cancelwait(ibuf);
            return true;
        } else if (waitres == strpartmatch) {
            return false;
        } else {
            if(++ibuf->start == INPUTBUF_DMA_RANGE) {
                ibuf->start = 0;
            }
            continue;
        }
    }
    return false; // this means always got strnotmatch at any position of received data.
}

// returns new chars read after previous call of this function
// line end char and delete char will not be included.
char* inputbuf_read_toline(struct inputbuf *ibuf, _Bool *lineend) {
    *lineend = false;
    if(ibuf->lineendmet) {
        inputbuf_clearline(ibuf);
    }
    char* result = ibuf->line + ibuf->linelen;
    if(!inputbuf_waitcheck(ibuf)) {
        return result;
    }
    while(ibuf->start != ibuf->end && ibuf->linelen < linelenlimit) {
        char next = ibuf->buf[ibuf->start++];
        ibuf->line[ibuf->linelen++] = next;
        if(ibuf->start == INPUTBUF_DMA_RANGE) {
            ibuf->start = 0;
        }
        // 关键要注意超过linelen的地方要及时清零。
        // line end char
        size_t lechcnt = sizeof(inputbuf_lineends) / sizeof(char);
        for(size_t i = 0; i < lechcnt; i++) {
            if(next == inputbuf_lineends[i]) {
                *lineend = ibuf->lineendmet = true;
                ibuf->line[--ibuf->linelen] = 0;
                break; // need to break inner and outer cycle
            }
        }
        if(*lineend) {
            break; // break outer cycle
        }
        // backspace or del char
        if(next == '\b' || next == 0x7f) {
            ibuf->line[--ibuf->linelen] = 0;
            if(ibuf->linelen) {
                ibuf->line[--ibuf->linelen] = 0;
            }
        }
    }
    if(ibuf->linelen == linelenlimit) {
        *lineend = ibuf->lineendmet = true;
    }
    return result;
}

// returns new chars after previous input line-end.
// call this after inputbuf_read() shows a line-end can read a full line input.
// call inputbuf_read() again after inputbuf_read() shows a line-end will clear line buffer.
char* inputbuf_getline(struct inputbuf *ibuf) {
    return ibuf->line;
}
// clear line buffer
void inputbuf_clearline(struct inputbuf *ibuf) {
    memset(ibuf->line, 0, ibuf->linelen);
    ibuf->linelen = 0;
    ibuf->lineendmet = false;
}


// raw模式下的阅读函数，适用于一些硬件模块不使用ASCII而使用二进制格式做串口通讯的场合。
// raw和line模式的阅读函数不能穿插混合使用，raw模式读完目标字节数之后才能再使用read_toline()。
// 通过多次调用这个函数可以得到想要阅读的巨大数目的内容。
// 比如我们的缓冲区只有100字节但是我想要读取1000个字节到数组里面。那么这个时候每次主循环里面调用这个函数，它会把新到的内容拼接到dest后面。
// 参数是函数结果存放区dest，需要读取的字节数targetcount比如1000，这个函数已经读入的字节数readcount。
// 函数的返回值是新的这个函数已经读取的字节数，返回值>=readcount。
// 如果返回值等于targetcount，那么已经读取完成；如果小于targetcount，那么下次调用这个函数的时候要把返回值作为第四个参数传入。
uint32_t inputbuf_read_raw(struct inputbuf *ibuf, char* dest, uint32_t targetcount, uint32_t readcount) {
    if(!inputbuf_waitcheck(ibuf)) {
        return readcount;
    }
    // 这个函数和用memcpy里面可能会大块搬运内存应该不会相差太多性能，不管了。
    while(ibuf->start != ibuf->end && readcount < targetcount) {
        dest[readcount++] = ibuf->buf[ibuf->start++];
        if(ibuf->start == INPUTBUF_DMA_RANGE)
            ibuf->start = 0;
    }
    return readcount;
}

// 屏蔽 read_toline() 和 read_raw() 两个函数读进内容，直到遇见了 keyword 才接触屏蔽。
void inputbuf_waitfor(struct inputbuf *ibuf, const char* keyword) {
    ibuf->waitforstr = keyword;
}
void inputbuf_cancelwait(struct inputbuf *ibuf) {
    ibuf->waitforstr = NULL;
}
_Bool inputbuf_iswaiting(struct inputbuf *ibuf) {
    return ibuf->waitforstr != NULL;
}


