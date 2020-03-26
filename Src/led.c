#include "led.h"
#include "main.h"
#include "log_uart.h"

#define LED_PWM_MODE TIM_OCMODE_PWM1
static TIM_HandleTypeDef *ledtim;
static uint32_t chr, chg, chb;


void led_init(TIM_HandleTypeDef *htim, uint32_t chred, uint32_t chgreen, uint32_t chblue) {
	HAL_TIM_PWM_Start(htim, chred);
	HAL_TIM_PWM_Start(htim, chgreen);
	HAL_TIM_PWM_Start(htim, chblue);
	ledtim = htim;
	chr=chred;
	chg=chgreen;
	chb=chblue;
}

void led_set(uint8_t r,uint8_t g,uint8_t b) {
	TIM_OC_InitTypeDef sconfig = {.OCMode = LED_PWM_MODE, .Pulse = r};
	HAL_TIM_PWM_ConfigChannel(ledtim, &sconfig, chr);
	sconfig.Pulse = g;
	HAL_TIM_PWM_ConfigChannel(ledtim, &sconfig, chg);
	sconfig.Pulse = b;
	HAL_TIM_PWM_ConfigChannel(ledtim, &sconfig, chb);
	logu_f(LOGU_INFO, "Led set to RGB: %hu %hu %hu", r, g, b);
}
