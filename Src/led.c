#include "led.h"
#include "main.h"

static uint8_t red = 0, green = 0, blue = 0;
static uint8_t counter = 0;

void led_set(uint8_t r,uint8_t g,uint8_t b) {
	red = r;
	green = g;
	blue = b;
}

void led_timer_int() {
	if(counter == 0) {
		LED_R_GPIO_Port->BSRR = LED_R_Pin;
		LED_G_GPIO_Port->BSRR = LED_G_Pin;
		LED_B_GPIO_Port->BSRR = LED_B_Pin;
	}
	if(counter == red)
		LED_R_GPIO_Port->BSRR = LED_R_Pin << 16;
	if(counter == green)
		LED_G_GPIO_Port->BSRR = LED_G_Pin << 16;
	if(counter == blue)
		LED_B_GPIO_Port->BSRR = LED_B_Pin << 16;
	counter++;
	// to split time into 255 parts which can be represented in light range from [0, 255], counter range is [0, 254].
	if(counter == 254)
		counter = 0;
}
