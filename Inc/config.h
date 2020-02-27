#ifndef __CONFIG_H__
#define __CONFIG_H__

#include "stm32f1xx.h"
/* 以下是设置项目，在config.c里还有一部分 Configurations Start */
// 串口
// 考虑到 115200 11.5B/ms 那么一个主循环的约10ms不会超过140B
// 现在的触发器有缓冲区一半填满，全部填满，和IDLE接受完成三个地方，
// 这些触发器会修改Flag在主循环里面通过检查flag再读入缓冲。
// 我们设置成两个主循环的大小不会丢失数据。
#define UART_INPUT_BUF_SIZE 300
// 串口命令长度设置
#define cmd_length_limit 40

// LED
#define LED1_GPIO GPIOA
#define LED1_GPIO_PIN GPIO_PIN_8
#define LED2_GPIO GPIOC
#define LED2_GPIO_PIN GPIO_PIN_9

// Tick Report
#define CYCLETICK_REPORT

/* 以上是设置项目 Configurations End */

#endif
