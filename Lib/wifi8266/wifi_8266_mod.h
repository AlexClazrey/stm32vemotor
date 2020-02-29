#ifndef __WIFI_8266_MOD_H__
#define __WIFI_8266_MOD_H__
#include "stm32f1xx_hal.h"

/* Configurations */
#define WIFI_ARGV_SIZE 3
#define WIFI_STACK_SIZE 5
#define WIFI_TASK_SIZE 20

#define WIFI_RECV_BUFFER_SIZE 500
#define WIFI_RECV_DMA_RANGE 495

// instruction return state, whether ok or error
typedef enum {
    WRS_OK,
    WRS_ERROR,
    WRS_INVALID,
    WRS_TIMEOUT,
    WRS_PASS,
    WRS_BUSY,
} WifiRtnState;

typedef struct wifi_handle Wifi_HandleTypeDef;
typedef WifiRtnState (*wifi_func)(Wifi_HandleTypeDef *hwifi, int timeisout, WifiRtnState subroutine);
typedef WifiRtnState (*wifi_func_withargs)(Wifi_HandleTypeDef *hwifi, int timeisout, WifiRtnState subroutine, int argc,
        int argv[WIFI_ARGV_SIZE]);
typedef int (*wifi_triggerfunc)(const char *recvstr);

struct wifi_stack_item {
    uint32_t deadline;
    WifiRtnState subroutine; // store result of subroutine
    wifi_triggerfunc trigger;
    union {
        wifi_func nextfunc;
        wifi_func_withargs nextargsfunc;
    };
    int argc;
    int argv[WIFI_ARGV_SIZE];
};

// empty ascending stack
struct wifi_stack {
    struct wifi_stack_item stack[WIFI_STACK_SIZE];
    size_t len;
};

struct wifi_recv {
    volatile int idle; // indicate whether receive process is finished
    volatile size_t len;
    char data[WIFI_RECV_BUFFER_SIZE];           // the data received
};

typedef WifiRtnState (*wifi_taskfunc)(Wifi_HandleTypeDef *hwifi);
typedef WifiRtnState (*wifi_taskfunc_withargs)(Wifi_HandleTypeDef *hwifi, int argc, int argv[WIFI_ARGV_SIZE]);
typedef void (*wifi_task_callback)(Wifi_HandleTypeDef *hwifi, WifiRtnState state, int taskindex, int taskfinished);

struct wifi_task_item {
    union {
        wifi_taskfunc func;
        wifi_taskfunc_withargs argsfunc;
    };
    int argc;
    int argv[WIFI_ARGV_SIZE];
    const char* name;
};
struct wifi_task {
    const char* tasks_name;
    struct wifi_task_item tasks[WIFI_TASK_SIZE];
    uint32_t cursor;
    uint32_t len;
};

// handle for wifi module esp8266
struct wifi_handle {
    UART_HandleTypeDef *huart;   // uart port that connected to the module
    struct wifi_task task;
    struct wifi_stack callstack;
    struct wifi_recv recv;
};

const char* rtntostr(WifiRtnState state);

void wifi_rx_idle_int(Wifi_HandleTypeDef *hwifi, DMA_HandleTypeDef* dmarx);
const char* wifi_rx_cap(Wifi_HandleTypeDef* hwifi);
size_t wifi_rx_cap_len(Wifi_HandleTypeDef* hwifi);
WifiRtnState wifi_tick(Wifi_HandleTypeDef *hwifi, wifi_task_callback callback);

size_t wifi_callstack_len(Wifi_HandleTypeDef *hwifi);
int wifi_callstack_isempty(Wifi_HandleTypeDef *hwifi);

void wifi_task_setlistname(Wifi_HandleTypeDef *hwifi, const char* name);
const char* wifi_task_getlistname(Wifi_HandleTypeDef *hwifi);
WifiRtnState wifi_task_add(Wifi_HandleTypeDef *hwifi, wifi_taskfunc taskfunc);
WifiRtnState wifi_task_add_withname(Wifi_HandleTypeDef *hwifi, wifi_taskfunc taskfunc, const char* name);
WifiRtnState wifi_task_add_withargs(Wifi_HandleTypeDef *hwifi, wifi_taskfunc_withargs taskfunc, const char* name, int argc, int argv[WIFI_ARGV_SIZE]);
const char* wifi_task_getitemname(Wifi_HandleTypeDef* hwifi, uint32_t index);
void wifi_task_clear(Wifi_HandleTypeDef *hwifi);
size_t wifi_task_len(Wifi_HandleTypeDef *hwifi);
int wifi_task_isempty(Wifi_HandleTypeDef *hwifi);
size_t wifi_task_remains(Wifi_HandleTypeDef *hwifi);
int wifi_task_isfinished(Wifi_HandleTypeDef *hwifi);

WifiRtnState wifi_checkat(Wifi_HandleTypeDef *hwifi);
WifiRtnState wifi_setmodewifi_client(Wifi_HandleTypeDef *hwifi);
WifiRtnState wifi_joinap(Wifi_HandleTypeDef *hwifi, const char *ssid, const char *password);
WifiRtnState wifi_joinap_args(Wifi_HandleTypeDef *hwifi, int argc, int *argv);
WifiRtnState wifi_leaveap(Wifi_HandleTypeDef *hwifi);
WifiRtnState wifi_scanap(Wifi_HandleTypeDef *hwifi);
WifiRtnState wifi_setsingleconn(Wifi_HandleTypeDef *hwifi);
WifiRtnState wifi_setmodetrans_unvarnished(Wifi_HandleTypeDef *hwifi);
WifiRtnState wifi_setmodetrans_normal(Wifi_HandleTypeDef *hwifi);
WifiRtnState wifi_tcpconn(Wifi_HandleTypeDef *hwifi, const char *ip, uint16_t port);
WifiRtnState wifi_tcpconn_args(Wifi_HandleTypeDef *hwifi, int argc, int *argv);
WifiRtnState wifi_dropsingleconn(Wifi_HandleTypeDef *hwifi);
WifiRtnState wifi_startsend(Wifi_HandleTypeDef *hwifi);
WifiRtnState wifi_stopsend(Wifi_HandleTypeDef *hwifi);
WifiRtnState wifi_task_delay(Wifi_HandleTypeDef* hwifi, int argc, int *argv);

HAL_StatusTypeDef wifi_send_str(Wifi_HandleTypeDef *hwifi, const char *data);
HAL_StatusTypeDef wifi_send_raw(Wifi_HandleTypeDef *hwifi, const char *buffer, size_t len);
WifiRtnState wifi_send_str_args(Wifi_HandleTypeDef *hwifi, int argc, int* argv);
WifiRtnState wifi_send_raw_args(Wifi_HandleTypeDef *hwifi, int argc, int* argv);

#endif
