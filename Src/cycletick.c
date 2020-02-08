#include "cycletick.h"
#include "util.h"
#include "log_uart.h"
#include <limits.h>

// tick value when a main cycle starts
static uint32_t tickstart = 0;

// ---------- Time
void cycle_tick_start() {
	tickstart = HAL_GetTick();
}

uint32_t cycle_tick_now() {
	return diffu(tickstart, HAL_GetTick(), UINT32_MAX + 1);
}

void cycle_tick_sleep_to(uint32_t ms) {
	uint32_t elapsed = cycle_tick_now();
	if (ms < elapsed) {
		log_uartf(LOGWARN, "Cycle Time Exceeded, time used: %d, target: %d.", elapsed, ms);
		return;
	}
	while (cycle_tick_now() < ms)
		;
}
