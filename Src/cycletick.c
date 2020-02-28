#include "cycletick.h"
#include "util.h"
#include "log_uart.h"
#include <limits.h>

/* Configurations */
#include "config.h"
// 是否开启汇报功能
#ifdef CYCLETICK_REPORT

// 每个几个周期汇报一次用时统计
#ifndef CYCLETICK_REPORT_CYCLE
#define CYCLETICK_REPORT_CYCLE 1000
#endif

#endif

/* Runtime Variables */
// tick value when a main cycle starts
static uint32_t tickstart = 0;

#ifdef CYCLETICK_REPORT
// stat report 
static uint32_t (*cyclecountfunc)() = NULL;
static uint32_t tickmax = 0;
static uint32_t ticksum = 0;

static void recordtime(uint32_t used);
static void reporttime();
#endif

// ---------- Time
void cycle_tick_start() {
	tickstart = HAL_GetTick();
}

uint32_t cycle_tick_now() {
	return diffu(tickstart, HAL_GetTick(), UINT32_MAX + 1);
}

void cycle_tick_sleep_to(uint32_t ms) {
	uint32_t elapsed = cycle_tick_now();
#ifdef CYCLETICK_REPORT
	recordtime(elapsed);
	if(cyclecountfunc && cyclecountfunc() % CYCLETICK_REPORT_CYCLE == 0) {
		reporttime();
	}
#endif
	if (ms < elapsed) {
		logu_f(LOGU_WARN, "Cycle Time Exceeded, time used: %d, target: %d.", elapsed, ms);
		return;
	}
	while (cycle_tick_now() < ms)
		;
}

#ifdef CYCLETICK_REPORT
// ---- stat report functions
void cycle_tick_init_report(uint32_t (*cyclecount)()) {
	cyclecountfunc = cyclecount;
}

static void recordtime(uint32_t used) {
	if(used > tickmax) {
		tickmax = used;
	}
	ticksum += used;
}

static void reporttime() {
	// avoid floating point as much as possible
	uint32_t aver = ticksum * 1000 / CYCLETICK_REPORT_CYCLE;
	uint32_t p1 = aver / 1000;
	uint32_t p2 = aver % 1000;

	logu_f(LOGU_DEBUG, "Cycle tick report: max: %lu, average: %lu.%lu", tickmax, p1, p2);
	ticksum = 0;
	tickmax = 0;
}
#else
void cycle_tick_init_report(uint32_t (*cyclecount)()) {
}
#endif
