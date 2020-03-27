#include "cycletick.h"
#include "util.h"
#include "log_uart.h"
#include <limits.h>

#define true 1
#define false 0




/* Runtime Variables */
static uint32_t cyclecount = 0;

// tick value when a main cycle starts
static uint32_t tickstart = 0;

static _Bool needreport = true;

#ifdef CYCLETICK_REPORT
static uint32_t tickmax = 0;
static uint32_t ticksum = 0;

static void recordtime(uint32_t used);
static void reporttime();
#endif

/* Functions */
// Time functions
void cycletick_start() {
	tickstart = HAL_GetTick();
}

uint32_t cycletick_now() {
	return diffu(tickstart, HAL_GetTick(), UINT32_MAX + 1);
}

void cycletick_sleepto(uint32_t ms) {
	uint32_t elapsed = cycletick_now();
#ifdef CYCLETICK_REPORT
	recordtime(elapsed);
	if(needreport && cyclecount % CYCLETICK_REPORT_CYCLE == 0) {
		reporttime();
	}
#endif
	if (ms < elapsed) {
		logu_f(LOGU_WARN, "Cycle Time Exceeded, time used: %d, target: %d.", elapsed, ms);
		return;
	}
	while (cycletick_now() < ms)
		;
}

void cycletick_sleeptoend() {
	cyclecount++;
	if(cyclecount == CYCLE_LIMIT) {
		cyclecount = 0;
	}
	cycletick_sleepto(CYCLE_INTV);
}

uint32_t cycletick_getcount() {
	return cyclecount;
}

_Bool cycletick_everyms(uint32_t interval) {
	if(cyclecount % (interval / CYCLE_INTV) == 0) {
		return true;
	} else {
		return false;
	}
}

void cycletick_disablereport() {
    needreport = false;
}
void cycletick_enablereport() {
    needreport = true;
}

#ifdef CYCLETICK_REPORT
// stat report functions
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
#endif
