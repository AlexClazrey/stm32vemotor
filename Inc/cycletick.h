#ifndef __CYCLETICK_H__
#define __CYCLETICK_H__

#include "stm32f1xx.h"

void cycle_tick_start();
uint32_t cycle_tick_now();
void cycle_tick_sleep_to(uint32_t ms);

#endif
