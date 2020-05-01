#ifndef __LED_H__
#define __LED_H__
#include <stdint.h>
#include "stm32f1xx.h"

struct rgb {
	uint8_t r,g,b;
};
void led_tick();
void led_init(TIM_HandleTypeDef *htim, uint32_t chred, uint32_t chgreen, uint32_t chblue);
void led_set(struct rgb *restrict color);
void led_get(struct rgb * restrict out_color);
void led_gradient_to(struct rgb *restrict to, uint32_t ms);
void led_gradient(struct rgb *restrict from, struct rgb *restrict to, uint32_t ms);

#endif

