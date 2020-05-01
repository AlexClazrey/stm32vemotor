#include "led.h"
#include "main.h"
#include "log_uart.h"
#include <string.h>

#define LED_PWM_MODE TIM_OCMODE_PWM1
static TIM_HandleTypeDef *ledtim;
static uint32_t chr, chg, chb;

_Bool is_grad = 0;
struct rgb grad_from, grad_to;
uint32_t grad_start, grad_now, grad_interval;

static void led_commit(uint8_t r, uint8_t g, uint8_t b);
static void led_calc_grad(struct rgb *restrict from, struct rgb *restrict to, uint32_t now, uint32_t interval);

void led_init(TIM_HandleTypeDef *htim, uint32_t chred, uint32_t chgreen, uint32_t chblue) {
	HAL_TIM_PWM_Start(htim, chred);
	HAL_TIM_PWM_Start(htim, chgreen);
	HAL_TIM_PWM_Start(htim, chblue);
	ledtim = htim;
	chr=chred;
	chg=chgreen;
	chb=chblue;
}

void led_tick() {
	if(is_grad) {
		grad_now = HAL_GetTick() - grad_start;
		if(grad_now <= grad_interval) {
			led_calc_grad(&grad_from, &grad_to, grad_now, grad_interval);
		} else {
			is_grad = 0;
			led_set(&grad_to);
		}
	}
}

void led_set(struct rgb* restrict color) {
	is_grad = 0;
	led_commit(color->r, color->g, color->b);
}

void led_get(struct rgb * restrict out_color) {
	uint32_t r = __HAL_TIM_GET_COMPARE(ledtim, chr);
	uint32_t g = __HAL_TIM_GET_COMPARE(ledtim, chg);
	uint32_t b = __HAL_TIM_GET_COMPARE(ledtim, chb);
	out_color->r = r;
	out_color->g = g;
	out_color->b = b;
}

void led_gradient_to(struct rgb *restrict to, uint32_t ms) {
	led_get(&grad_from);
	led_gradient(&grad_from, to, ms);
}

void led_gradient(struct rgb *restrict from, struct rgb *restrict to, uint32_t ms) {
	grad_from = *from;
	grad_to = *to;
	grad_interval = ms;
	grad_start = HAL_GetTick();
	grad_now = 0;
	is_grad = 1;
}

static void led_commit(uint8_t r,uint8_t g,uint8_t b) {
	TIM_OC_InitTypeDef sconfig = {.OCMode = LED_PWM_MODE, .Pulse = r};
	HAL_TIM_PWM_ConfigChannel(ledtim, &sconfig, chr);
	sconfig.Pulse = g;
	HAL_TIM_PWM_ConfigChannel(ledtim, &sconfig, chg);
	sconfig.Pulse = b;
	HAL_TIM_PWM_ConfigChannel(ledtim, &sconfig, chb);
	HAL_TIM_PWM_Start(ledtim, chr);
	HAL_TIM_PWM_Start(ledtim, chg);
	HAL_TIM_PWM_Start(ledtim, chb);
	logu_f(LOGU_TRACE, "Led set to RGB: %hu %hu %hu", r, g, b);
}

static void led_calc_grad(struct rgb *restrict from, struct rgb *restrict to, uint32_t now, uint32_t interval) {
	uint32_t rd, gd, bd;
	rd = to->r - from->r;
	gd = to->g - from->g;
	bd = to->b - from->b;
	rd = rd * now / interval;
	gd = gd * now / interval;
	bd = bd * now / interval;
	rd += from->r;
	gd += from->g;
	bd += from->b;
	led_commit(rd, gd, bd);
}



