#include "cycletick.h"
#include "util.h"
#include "config.h"
#include "log_uart.h"
#ifdef CYCLETICK_REPORT
#include "main.h"
#endif
#include <limits.h>

// tick value when a main cycle starts
static uint32_t tickstart = 0;

#ifndef CYCLETICK_REPORT_CYCLE
#define CYCLETICK_REPORT_CYCLE 1000
#endif
static uint32_t tickmax = 0;
static uint32_t ticksum = 0;

static void recordtime(uint32_t used);
static void reporttime();

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
	if(maincyclecount() % CYCLETICK_REPORT_CYCLE == 0) {
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
