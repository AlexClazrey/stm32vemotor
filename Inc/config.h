#ifndef __CONFIG_H__
#define __CONFIG_H__

#include "stm32f1xx.h"
/* 以下是设置项目，在config.c里还有一部分 Configurations Start */

// WIFI 模块是否加载
#define WIFI_ENABLE 1

// 初始化的时候要做的事情。
#define INIT_MOTOR_MOVE 0
#define INIT_WIFI_CONNECT 1

// WiFi每十秒发送一句问好
#define WIFI_GREET 0

// WiFi Length Limit
#define WIFI_STRSIZE 40

// Tick Report
//#define CYCLETICK_REPORT
// Log Report
//#define LOG_TXCPLT_REPORT

extern uint16_t machine_id;
extern char wifi_conf_ssid[WIFI_STRSIZE];
extern char wifi_conf_pwd[WIFI_STRSIZE];
extern char wifi_conf_tcpip[WIFI_STRSIZE];
extern uint16_t wifi_conf_tcpport;

extern int32_t lm_conf_limit_out;
extern int32_t lm_conf_limit_in;

// 电机循环测试配置
extern int lm_cycle_out;
extern int lm_cycle_in;
// 1的时候只有跑完一整个测试循环，点击在最里面的位置才会暂停一会儿，
// 0的时候在测试里的每一步都会暂停一会儿。
extern int lm_cycle_pause_at_full_cycle;
extern int lm_cycle_step_pause;


/* 以上是设置项目 Configurations End */

#endif
