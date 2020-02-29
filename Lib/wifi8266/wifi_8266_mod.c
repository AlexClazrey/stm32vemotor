#include "wifi_8266_mod.h"
#include "log_uart.h"
#include <string.h>
#include <stdio.h>

/* Configurations */
#ifndef TIMEOUTSHORT
#define TIMEOUTSHORT 1000
#endif

#ifndef TIMEOUTMID
#define TIMEOUTMID 4000
#endif

#ifndef TIMEOUTLONG
#define TIMEOUTLONG 15000
#endif

/*
 ======= 模块说明 =======
 这个模块对 ESP8266 AT指令 需要等待相应这部分做了异步化的处理。
 异步化中间的状态信息保存在 struct wifi_handle . struct wifi_stack 结构体里面。
 在每一次主循环的时候你需要调用 wifi_tick 函数对这部分信息更新。
 更新的时候会根据UART收到的内容和超时情况作出相应的操作。
 当异步化信息清空的时候（wifi_stack_isempty），表示异步操作完成，
 这个时候 wifi_tick 函数会返回结果，这个结果可以不看，因为同时这个函数参数里的回调会被触发。
 使用上面所说的功能，你能够一次执行一个AT指令，等待回调之后再执行下一个AT指令。
 如果不等待回调就做下一个AT，那么 ESP8266 很有可能会给你 busy 的反馈。

 那么想要一次顺序执行多个AT指令怎么办呢？
 在 struct wifi_stack 的异步化结构外，还有一层结构 struct wifi_task 。
 这层结构保存AT指令的队列信息。
 在 tick 的回调函数里面，
 第一个参数是 struct wifi_handler 对象自己，
 第二个参数是AT指令的结果，
 第三个参数是目前返回结果的 task 序号，如果没有使用 task 系统那么是 -1 ，
 第四个参数是 task 队列是否全部完成的 bool ，如果没有使用 task 系统那么是1/True。
 一个 task 失败不会终止后续的 task 执行，如果想要中止，那么在回调函数里面使用 wifi_task_clear ，
 因为回调函数在后续的 task 开始之前被触发。

 task序列和单独的task项目都可以设置名字，用这个在回调函数里面可以分辨来源，如果没有使用那么都是NULL。

 ESP-01 模块的功能在透传开启的时候，收到的TCP数据会直接输出，透传关闭的时候收到数据会有 +IPD,<len>:<data> 。 
 关闭传输模式的语句 "+++" 和其他信息之间至少间隔100ms，在task队列里面使用wifi_task_delay函数可以做到延时。

 速记：
 在主循环里调用 wifi_tick 填入wifi_handle和AT指令完成的回调函数。
 在 stm32f1xx_it.c 里面收到 IDLE 中断的时候调用 wifi_rx_idle_int 。
 不使用 task 队列发起AT指令只要调用对应的函数就行。
 使用 task 队列的话调用 wifi_task_add 或者 wifi_task_add_withargs 添加任务。
 在使用 wifi_startsend 开启传输之后 wifi_send_raw/str 可以写入数据，完成之后使用 wifi_stopsend 退出传输模式。
 在 wifi_tick 之前调用 wifi_rx_cap 函数可以得到原始的接受字符串。
 wifi_tick 不会把接受到的数据保留到下一次 wifi_tick 里面使用。

 */

/*
 ======= ISSUES =======
 1. ESP8266 太长时间的放置之后还会没反应那，这怎么办呢？还要定期AT一下吗？如果没反应就HARD RESET？
 而且这种时候我手工 HARD RESET 多次才有用。
 2. 某些时刻，在发生外部事件的时候，
 会有 WIFI CONNECTED / WIFI GOT IP / WIFI DISCONNECT 这样的字符输出；
 另一些时刻没有，这个和AT+CIPMODE设置有关。
 如果=1那么只有TCP上面的事件 CONNECT / CLOSE会输出。
 如果=0那么TCP和WIFI事件都会有输出。
 3.	连接 WiFi 的时候有一定机率引发它一直处于BUSY状态很久很久很久几千秒也不会停。
 它不会相应任何新的AT指令，全都返回busy，我没法中断连接。
 所以这种情况也要 RESET ，我不知道 AT+RESET 有没有用。
 看起来真的很需要一个GPIO单独连接到RESET上。
 4.	在 Reset 之后最后的输出也不一定是 ready\r\n 才能表示正常启动，关键判别在于AT有反应。
 这可能要几次 RESET ，网上说这是因为ESP8266的电源要求很高，要很稳定。
 但是在单独的试验电路里面我加了2000uF的固态电容就没事了。
 最好单片机初始化的时候就RESET试试。

 */

/* Runtime Variables */

static const char *RTNOK = "\r\nOK\r\n";
static const char *RTNERR = "\r\nERROR\r\n";
// 8266 prints "busy p...\r\n" if you give a another command while processing previous one.
static const char *RTNBUSY = "\r\nbusy ";

static char *rtnstrarr[6] = { "OK", "Error", "Invalid", "Timeout", "Pass", "Busy" };
/* Functions */
/* ---- Macros ---- */
#define CHECKSENT       \
    if (sent != HAL_OK) \
        return WRS_ERROR;

#define THROWTIMEOUT \
    if (timeisout)   \
        return WRS_TIMEOUT;

// to suppress unused local function warnings.
#ifdef __GNUC__
#define UNUSED_ATTRIB __attribute__((unused))
#else
#define UNUSED_ATTRIB
#endif

/* ---- Utilities ---- */
static inline int strhas(const char *from, const char *target) {
    return strstr(from, target) != NULL;
}
static inline int checkok(const char *str) {
    return strhas(str, RTNOK);
}
static inline int checkerror(const char *str) {
    return strhas(str, RTNERR);
}
static inline int checkbusy(const char *str) {
    return strhas(str, RTNBUSY);
}
static inline int UNUSED_ATTRIB checkhasrecv(struct wifi_recv *recv, const char *target) {
    return strhas(recv->data, target);
}
static inline WifiRtnState recvstrsignal(const char *str) {
    return checkok(str) ? WRS_OK : checkerror(str) ? WRS_ERROR : checkbusy(str) ? WRS_BUSY : WRS_INVALID;
}

const char* rtntostr(WifiRtnState state) {
    return rtnstrarr[(int) state];
}

HAL_StatusTypeDef wifi_send_str(Wifi_HandleTypeDef *hwifi, const char *data) {
//    logu_f(LOGU_TRACE, "wifi send:\r\n%s", data);
    return HAL_UART_Transmit_DMA(hwifi->huart, (uint8_t*) data, strlen(data));
}

HAL_StatusTypeDef wifi_send_raw(Wifi_HandleTypeDef *hwifi, const char *buffer, size_t len) {
    return HAL_UART_Transmit_DMA(hwifi->huart, (uint8_t*) buffer, len);
}


/* ---- Wifi Rx Interrupt Routine Functions ---- */

// Call this function in UART Interrupt. You need to enable IDLE interrupt first.
void wifi_rx_idle_int(Wifi_HandleTypeDef *hwifi, DMA_HandleTypeDef* dmarx) {
    if (__HAL_UART_GET_FLAG(hwifi->huart, USART_SR_IDLE)) {
        __HAL_UART_CLEAR_FLAG(hwifi->huart, USART_SR_IDLE);
        HAL_UART_DMAStop(hwifi->huart);
        uint32_t dmacnt = __HAL_DMA_GET_COUNTER(dmarx);
        hwifi->recv.len = WIFI_RECV_DMA_RANGE - dmacnt;
        hwifi->recv.idle = 1;
        HAL_UART_Receive_DMA(hwifi->huart, (uint8_t*) hwifi->recv.data, WIFI_RECV_DMA_RANGE);
    }
}

/* ---- Wifi Rx Functions ---- */
const char* wifi_rx_cap(Wifi_HandleTypeDef* hwifi) {
    if(hwifi->recv.idle) {
        hwifi->recv.data[hwifi->recv.len] = 0;
        return hwifi->recv.data;
    }
    return NULL;
}
size_t wifi_rx_cap_len(Wifi_HandleTypeDef* hwifi) {
    return hwifi->recv.idle ? hwifi->recv.len : 0;
}

/* ---- Wifi Callstack Frame Functions ---- */
static int wifi_tick_frame_go_on(Wifi_HandleTypeDef *hwifi, struct wifi_stack_item *sti, WifiRtnState *out_result);
static int wifi_tick_frame_timeout(Wifi_HandleTypeDef *hwifi, struct wifi_stack_item *sti, WifiRtnState *out_result);
static WifiRtnState wifi_task_run(Wifi_HandleTypeDef *hwifi, struct wifi_task_item *task);

static WifiRtnState wifi_tick_p1(Wifi_HandleTypeDef *hwifi);
static WifiRtnState wifi_tick_p2(Wifi_HandleTypeDef *hwifi, wifi_task_callback callback, WifiRtnState from_p1);

WifiRtnState wifi_tick(Wifi_HandleTypeDef *hwifi, wifi_task_callback callback) {
    WifiRtnState result;
    result = wifi_tick_p1(hwifi);
    result = wifi_tick_p2(hwifi, callback, result);
    return result;
}

// 这段处理 callstack 的事情
static WifiRtnState wifi_tick_p1(Wifi_HandleTypeDef *hwifi) {
    struct wifi_stack *st = &hwifi->callstack;
    // 这是为了加速，没有这句判断一样能运行
    if (st->len == 0) {
        hwifi->recv.idle = 0;
        hwifi->recv.len = 0;
        return WRS_PASS;
    }
    WifiRtnState subroutine;
    WifiRtnState result = WRS_PASS;
    // 先从外向内扫描是不是完成
    while (st->len > 0) {
        st->len--;
        int res = wifi_tick_frame_go_on(hwifi, st->stack + st->len, &subroutine);
        if (res == 0) {
            st->len++;
            break;
        }
//        logu_f(LOGU_TRACE, "wifi stack len after run: %d, func: %lu", st->len, (uint32_t) st->stack[st->len].nextfunc);
        // 如果是底层那么弹出结果，这个结果在主循环或者回调里面处理。
        if (st->len == 0)
            result = subroutine;
        else
            st->stack[st->len - 1].subroutine = subroutine;
    }
    // 然后再扫描一遍是不是需要超时弹出
    int deepest_timeout;
    uint32_t tick = HAL_GetTick();
    for (deepest_timeout = 0; deepest_timeout < st->len; deepest_timeout++) {
        if (st->stack[deepest_timeout].deadline < tick) {
            break;
        }
    }
    // 如果有超时弹出的话，那么从外层到内层通知这个函数超时。
    for (; st->len > deepest_timeout; st->len--) {
        wifi_tick_frame_timeout(hwifi, st->stack + st->len - 1, &subroutine);
        if (st->len == 1)
            result = subroutine;
        else
            st->stack[st->len - 2].subroutine = subroutine;
    }
    // assert 一些不该发生的情况
    if (result != WRS_PASS && !wifi_callstack_isempty(hwifi)) {
        logu_s(LOGU_ERROR, "BUG. Wifi tick returns while wifi callstack is not empty.");
    }
    return result;
}

void wifi_task_clear(Wifi_HandleTypeDef *hwifi);
// 这段处理 task 的事情
static WifiRtnState wifi_tick_p2(Wifi_HandleTypeDef *hwifi, wifi_task_callback callback, WifiRtnState from_p1) {
    WifiRtnState result = from_p1;
    // 如果没有在 callstack 里面 pending 的操作那么执行下一个 task
    // 进入这里的时候有两种可能，一种是 result = WRS_PASS 一种是 result 包含了前一个函数的异步返回结果
    // 如果每个Task要向外界汇报结果的话，那么在result不是PASS的时候这里不能覆盖。
    if (result == WRS_PASS && wifi_callstack_isempty(hwifi) && !wifi_task_isfinished(hwifi)) {
        WifiRtnState taskresult = wifi_task_run(hwifi, &hwifi->task.tasks[hwifi->task.cursor++]);
        if (wifi_callstack_isempty(hwifi)) {
            // 如果这个task瞬间完成或者瞬间出错，那么从这里返回结果，不能丢失汇报
            result = taskresult;
            // 这种情况也会走到下面的回调上，所以这里不使用回调。
        }
    }

    // 如果有返回的结果，那么这是（上一个task 或者 没有使用task队列单独的一个函数调用的）异步执行结果，在这里调用回调函数
    // 如果有返回的结果，并且task队列已完成，那么需要清空队列信息。
    if (result != WRS_PASS) {
        // 在不使用task队列功能的时候cursor应该是0。
        callback(hwifi, result, hwifi->task.cursor - 1, wifi_task_isfinished(hwifi));
        if (wifi_task_isfinished(hwifi)) {
            wifi_task_clear(hwifi);
        }
    }

    return result;
}

static WifiRtnState wifi_frame_run(Wifi_HandleTypeDef *hwifi, int timeout, struct wifi_stack_item *sti) {
    if (sti->nextargsfunc)
        return sti->nextargsfunc(hwifi, timeout, sti->subroutine, sti->argc, sti->argv);
    else
        return sti->nextfunc(hwifi, timeout, sti->subroutine);
}

// 通过自定义触发函数可以处理 recv 收到的事件信息和AT指令结果信息的分离。
// 为什么要分离呢？
// 因为如果不分离堆栈的触发规则是收到信息就触发下一个函数，如果在预定的收到的事件数据之外收到了其他事件数据
// 那么就有可能让堆栈乱套，就假设一个突如其来的disconnect事件怎么处理？
// 是不是在收到输入的时候首先要先经过一轮Parse分成事件类和信号类两种，
// 这样也可以简化一条指令收到的返回信号的复杂判断。
// 破坏堆栈的本质原因是事件类的输出是不遵守FILO原理的，而应该当成一个FIFO队列来处理。

/*
 * return 0 for pass
 * return 1 for stack pop
 */
static int wifi_tick_frame_go_on(Wifi_HandleTypeDef *hwifi, struct wifi_stack_item *sti, WifiRtnState *out_result) {
    if (hwifi->recv.idle) {
        int result = 0;
        hwifi->recv.data[hwifi->recv.len] = 0;
        if (sti->trigger == NULL || sti->trigger(hwifi->recv.data)) {
            *out_result = wifi_frame_run(hwifi, 0, sti);
            result = 1;
        }
        hwifi->recv.idle = 0;
        hwifi->recv.len = 0;
        return result;
    }
    return 0;
}

/* always returns zero */
static int wifi_tick_frame_timeout(Wifi_HandleTypeDef *hwifi, struct wifi_stack_item *sti, WifiRtnState *out_result) {
    *out_result = wifi_frame_run(hwifi, 1, sti);
    return 0;
}

static WifiRtnState wifi_frame_add_withargs(Wifi_HandleTypeDef *hwifi, wifi_func_withargs nextfunc,
        uint32_t deadline, wifi_triggerfunc trigger, int argc, int argv[WIFI_ARGV_SIZE]) {
    if (argc > WIFI_ARGV_SIZE) {
        return WRS_ERROR;
    }
    struct wifi_stack *st = &hwifi->callstack;
    if (st->len == WIFI_STACK_SIZE) {
        return WRS_ERROR;
    }
    struct wifi_stack_item *sti = &st->stack[st->len++];
    sti->deadline = deadline;
    sti->nextargsfunc = nextfunc;
    sti->argc = argc;
    memcpy(sti->argv, argv, argc * sizeof(int));
    sti->subroutine = WRS_PASS;
    sti->trigger = trigger;
    return WRS_OK;
}

static WifiRtnState wifi_frame_add(Wifi_HandleTypeDef *hwifi, wifi_func nextfunc, uint32_t deadline,
        wifi_triggerfunc trigger) {
    return wifi_frame_add_withargs(hwifi, (wifi_func_withargs) nextfunc, deadline, trigger, 0, NULL);
}

size_t wifi_callstack_len(Wifi_HandleTypeDef *hwifi) {
    return hwifi->callstack.len;
}

int wifi_callstack_isempty(Wifi_HandleTypeDef *hwifi) {
    return wifi_callstack_len(hwifi) == 0;
}

/* ---- Task Functions ---- */
WifiRtnState wifi_task_add(Wifi_HandleTypeDef *hwifi, wifi_taskfunc taskfunc) {
    return wifi_task_add_withargs(hwifi, (wifi_taskfunc_withargs) taskfunc, NULL, 0, NULL);
}

WifiRtnState wifi_task_add_withname(Wifi_HandleTypeDef *hwifi, wifi_taskfunc taskfunc, const char* name) {
    return wifi_task_add_withargs(hwifi, (wifi_taskfunc_withargs) taskfunc, name, 0, NULL);
}

WifiRtnState wifi_task_add_withargs(Wifi_HandleTypeDef *hwifi, wifi_taskfunc_withargs taskfunc,
        const char* name, int argc, int argv[WIFI_ARGV_SIZE]) {
    if (argc > WIFI_ARGV_SIZE) {
        return WRS_ERROR;
    }
    struct wifi_task *pt = &hwifi->task;
    if (pt->len == WIFI_TASK_SIZE) {
        return WRS_ERROR;
    }
    struct wifi_task_item *pti = &pt->tasks[pt->len++];
    pti->argsfunc = taskfunc;
    pti->argc = argc;
    memcpy(pti->argv, argv, argc * sizeof(int));
    pti->name = name;
    return WRS_OK;
}

const char* wifi_task_getitemname(Wifi_HandleTypeDef* hwifi, uint32_t index) {
    if(index >= hwifi->task.len) 
        return NULL;
    else
        return hwifi->task.tasks[index].name;
}

void wifi_task_setlistname(Wifi_HandleTypeDef *hwifi, const char* name) {
    hwifi->task.tasks_name = name;
}

const char* wifi_task_getlistname(Wifi_HandleTypeDef *hwifi) {
    return hwifi->task.tasks_name;
}

void wifi_task_clear(Wifi_HandleTypeDef *hwifi) {
    hwifi->task.tasks_name = NULL;
    hwifi->task.len = 0;
    hwifi->task.cursor = 0;
}

size_t wifi_task_len(Wifi_HandleTypeDef *hwifi) {
    return hwifi->task.len;
}

int wifi_task_isempty(Wifi_HandleTypeDef *hwifi) {
    return wifi_task_len(hwifi) == 0;
}

size_t wifi_task_remains(Wifi_HandleTypeDef *hwifi) {
    return hwifi->task.len - hwifi->task.cursor;
}

int wifi_task_isfinished(Wifi_HandleTypeDef *hwifi) {
    return wifi_task_remains(hwifi) == 0;
}

static WifiRtnState wifi_task_run(Wifi_HandleTypeDef *hwifi, struct wifi_task_item *task) {
    if (task == NULL) {
        logu_s(LOGU_ERROR, "Wifi task function is null.");
        return WRS_ERROR;
    }
    if (task->argc == 0)
        return task->func(hwifi);
    else
        return task->argsfunc(hwifi, task->argc, task->argv);
}

/* ---- User Functions ---- */
static int wifi_trigger_atsignal(const char *data) {
    return recvstrsignal(data) != WRS_INVALID;
}
// 这可以做到一个延时层的效果。
static int wifi_trigger_none(const char* data) {
    return 0;
}

static WifiRtnState wifi_dummyframe(Wifi_HandleTypeDef *hwifi, int timeisout, WifiRtnState subroutine) {
    return WRS_OK;
}

static WifiRtnState wifi_checkrecvsignal(Wifi_HandleTypeDef *hwifi, int timeisout, WifiRtnState subroutine) {
    THROWTIMEOUT;
    return recvstrsignal(hwifi->recv.data);
}

WifiRtnState wifi_checkat(Wifi_HandleTypeDef *hwifi) {
    const char *inst = "AT\r\n";
    HAL_StatusTypeDef sent = wifi_send_str(hwifi, inst);
    CHECKSENT;
    wifi_frame_add(hwifi, wifi_checkrecvsignal, HAL_GetTick() + TIMEOUTSHORT, wifi_trigger_atsignal);
    return WRS_OK;
}

WifiRtnState wifi_setmodewifi_client(Wifi_HandleTypeDef *hwifi) {
    const char *inst = "AT+CWMODE=1\r\n";
    HAL_StatusTypeDef sent = wifi_send_str(hwifi, inst);
    CHECKSENT;
    wifi_frame_add(hwifi, wifi_checkrecvsignal, HAL_GetTick() + TIMEOUTSHORT, wifi_trigger_atsignal);
    return WRS_OK;
}

static WifiRtnState wifi_joinap_2(Wifi_HandleTypeDef *hwifi, int timeisout, WifiRtnState subroutine);
char joinbuf[100];
// 这个很奇特，命令和参数必须分开时间发送。
WifiRtnState wifi_joinap(Wifi_HandleTypeDef *hwifi, const char *ssid, const char *password) {
    const char *inst = "AT+CWJAP=";
    int len = snprintf(joinbuf, 100, "\"%s\",\"%s\"\r\n", ssid, password);
    if (len < 0 || len >= 100)
        return WRS_ERROR;
    HAL_StatusTypeDef sent = wifi_send_str(hwifi, inst);
    CHECKSENT;
    wifi_frame_add(hwifi, wifi_joinap_2, HAL_GetTick() + TIMEOUTSHORT, NULL);
    return WRS_OK;
}
// 对上面一个函数的task形式的接口。
WifiRtnState wifi_joinap_args(Wifi_HandleTypeDef *hwifi, int argc, int *argv) {
    if (argc != 2)
        return WRS_ERROR;
    return wifi_joinap(hwifi, (const char*) argv[0], (const char*) argv[1]);
}

static WifiRtnState wifi_joinap_2(Wifi_HandleTypeDef *hwifi, int timeisout, WifiRtnState subroutine) {
    THROWTIMEOUT;
    HAL_StatusTypeDef sent = wifi_send_str(hwifi, joinbuf);
    CHECKSENT;
    wifi_frame_add(hwifi, wifi_checkrecvsignal, HAL_GetTick() + TIMEOUTLONG, wifi_trigger_atsignal);
    return WRS_OK;
}

WifiRtnState wifi_leaveap(Wifi_HandleTypeDef *hwifi) {
    const char *inst = "AT+CWQAP\r\n";
    HAL_StatusTypeDef sent = wifi_send_str(hwifi, inst);
    CHECKSENT;
    // 用了过滤机制之后不需要担心什么时候输出 DISCONNECT 这句话了。
    wifi_frame_add(hwifi, wifi_checkrecvsignal, HAL_GetTick() + TIMEOUTSHORT, wifi_trigger_atsignal);
    return WRS_OK;
}

WifiRtnState wifi_scanap(Wifi_HandleTypeDef *hwifi) {
    const char *inst = "AT+CWLAP\r\n";
    HAL_StatusTypeDef sent = wifi_send_str(hwifi, inst);
    CHECKSENT;
    wifi_frame_add(hwifi, wifi_checkrecvsignal, HAL_GetTick() + TIMEOUTLONG, wifi_trigger_atsignal);
    return WRS_OK;
}
// TODO scan result parse function

WifiRtnState wifi_setsingleconn(Wifi_HandleTypeDef *hwifi) {
    const char *inst = "AT+CIPMUX=0\r\n";
    HAL_StatusTypeDef sent = wifi_send_str(hwifi, inst);
    CHECKSENT;
    wifi_frame_add(hwifi, wifi_checkrecvsignal, HAL_GetTick() + TIMEOUTSHORT, wifi_trigger_atsignal);
    return WRS_OK;
}
WifiRtnState wifi_setmodetrans_unvarnished(Wifi_HandleTypeDef *hwifi) {
    const char *inst = "AT+CIPMODE=1\r\n";
    HAL_StatusTypeDef sent = wifi_send_str(hwifi, inst);
    CHECKSENT;
    wifi_frame_add(hwifi, wifi_checkrecvsignal, HAL_GetTick() + TIMEOUTSHORT, wifi_trigger_atsignal);
    return WRS_OK;
}
WifiRtnState wifi_setmodetrans_normal(Wifi_HandleTypeDef *hwifi) {
    const char *inst = "AT+CIPMODE=0\r\n";
    HAL_StatusTypeDef sent = wifi_send_str(hwifi, inst);
    CHECKSENT;
    wifi_frame_add(hwifi, wifi_checkrecvsignal, HAL_GetTick() + TIMEOUTSHORT, wifi_trigger_atsignal);
    return WRS_OK;
}

static WifiRtnState wifi_tcpconn_2(Wifi_HandleTypeDef *hwifi, int timeisout, WifiRtnState subroutine);
char connbuf[100];
// 这个也很奇特，命令和参数必须分开时间发送。
WifiRtnState wifi_tcpconn(Wifi_HandleTypeDef *hwifi, const char *ip, uint16_t port) {
    const char *inst = "AT+CIPSTART=";
    size_t len = snprintf(connbuf, 100, "\"TCP\",\"%s\",%hu\r\n", ip, port);
    if (len < 0 || len >= 100)
        return WRS_ERROR;
    HAL_StatusTypeDef sent = wifi_send_str(hwifi, inst);
    CHECKSENT;
    wifi_frame_add(hwifi, wifi_tcpconn_2, HAL_GetTick() + TIMEOUTSHORT, NULL);
    return WRS_OK;
}
WifiRtnState wifi_tcpconn_args(Wifi_HandleTypeDef *hwifi,int argc, int* argv) {
    if (argc != 2) {
        return WRS_ERROR;
    }
    return wifi_tcpconn(hwifi, (char*)argv[0], (uint16_t)argv[1]);
}

static WifiRtnState wifi_tcpconn_2(Wifi_HandleTypeDef *hwifi, int timeisout, WifiRtnState subroutine) {
    THROWTIMEOUT;
    HAL_StatusTypeDef sent = wifi_send_str(hwifi, connbuf);
    CHECKSENT;
    wifi_frame_add(hwifi, wifi_checkrecvsignal, HAL_GetTick() + TIMEOUTMID, wifi_trigger_atsignal);
    return WRS_OK;
}

WifiRtnState wifi_dropsingleconn(Wifi_HandleTypeDef *hwifi) {
    const char *inst = "AT+CIPCLOSE\r\n";
    HAL_StatusTypeDef sent = wifi_send_str(hwifi, inst);
    CHECKSENT;
    wifi_frame_add(hwifi, wifi_checkrecvsignal, HAL_GetTick() + TIMEOUTSHORT, wifi_trigger_atsignal);
    return WRS_OK;
}

WifiRtnState wifi_startsend(Wifi_HandleTypeDef *hwifi) {
    const char *inst = "AT+CIPSEND\r\n";
    HAL_StatusTypeDef sent = wifi_send_str(hwifi, inst);
    CHECKSENT;
    wifi_frame_add(hwifi, wifi_checkrecvsignal, HAL_GetTick() + TIMEOUTSHORT, wifi_trigger_atsignal);
    return WRS_OK;
}

WifiRtnState wifi_stopsend(Wifi_HandleTypeDef *hwifi) {
    const char *inst = "+++";
    HAL_StatusTypeDef sent = wifi_send_str(hwifi, inst);
    CHECKSENT;
    // delay 100ms 
    wifi_frame_add(hwifi, wifi_dummyframe, HAL_GetTick() + 100, wifi_trigger_none);
    return WRS_OK;
}

// argv[0] is uint32_t delay in ms.
WifiRtnState wifi_task_delay(Wifi_HandleTypeDef* hwifi, int argc, int *argv) {
    if (argc != 1) {
        return WRS_ERROR;
    }
    wifi_frame_add(hwifi, wifi_dummyframe, HAL_GetTick() + (uint32_t)(argv[0]), wifi_trigger_none);
    return WRS_OK;
}

// argv[0] is str to send
WifiRtnState wifi_send_str_args(Wifi_HandleTypeDef *hwifi, int argc, int* argv) {
    if (argc != 1) {
        return WRS_ERROR;
    }
    return wifi_send_str(hwifi, (char*)argv[0]) == HAL_OK ? WRS_OK : WRS_ERROR;
}

// argv[0] is buffer, argv[1] is len.
WifiRtnState wifi_send_raw_args(Wifi_HandleTypeDef *hwifi, int argc, int* argv) {
    if (argc != 2) {
        return WRS_ERROR;
    }
    return wifi_send_raw(hwifi, (char*)argv[0], (size_t)argv[1]) == HAL_OK ? WRS_OK : WRS_ERROR;
}
