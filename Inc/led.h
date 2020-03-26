#ifndef __LED_H__
#define __LED_H__
#include <stdint.h>
#include "stm32f1xx.h"

void led_init(TIM_HandleTypeDef *htim, uint32_t chred, uint32_t chgreen, uint32_t chblue);
void led_set(uint8_t r,uint8_t g,uint8_t b);

#endif

