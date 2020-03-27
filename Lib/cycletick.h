#ifndef __CYCLETICK_H__
#define __CYCLETICK_H__

#include "stm32f1xx.h"

/* Configurations Start */
#include "config.h"

#ifndef CYCLE_INTV
#define CYCLE_INTV 10
#endif

#ifndef CYCLE_LIMIT
#define CYCLE_LIMIT 10000
#endif 

// 是否开启汇报功能
#ifdef CYCLETICK_REPORT
// 每个几个周期汇报一次用时统计
#ifndef CYCLETICK_REPORT_CYCLE
#define CYCLETICK_REPORT_CYCLE 1000
#endif
#endif

/* Configurations End */

void cycletick_start();
uint32_t cycletick_now();
uint32_t cycletick_getcount();
_Bool cycletick_everyms(uint32_t interval);
void cycletick_sleepto(uint32_t ms);
void cycletick_sleeptoend();
void cycletick_disablereport();
void cycletick_enablereport();

#endif
