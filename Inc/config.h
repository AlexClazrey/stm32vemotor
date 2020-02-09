#ifndef __CONFIG_H__
#define __CONFIG_H__

#include "stm32f1xx.h"
/* 以下是设置项目，在config.c里还有一部分 Configurations Start */

// 这个是机器编号，只能是一个字节。
extern uint8_t machine_id;

// 移动到比例位置的时候使用的范围。
extern const int lm_limit_out;
extern const int lm_limit_in;

// 电机循环测试
extern const int lm_cycle_out;
extern const int lm_cycle_in;
// 1的时候只有跑完一整个测试循环，点击在最里面的位置才会暂停一会儿，
// 0的时候在测试里的每一步都会暂停一会儿。
extern int lm_cycle_pause_at_full_cycle;

// CAN 命令参数
extern const uint8_t CAN_CMD_VER;
extern const uint8_t CAN_CMD_LM;
extern const uint8_t CAN_CMD_STRING;

// 主循环时间设置
extern const uint32_t COUNT_INTV;
extern const uint32_t COUNT_LIMIT;

// 串口
// 考虑到 115200 11.5B/ms 那么一个主循环的约10ms不会超过140B
// 现在的触发器有缓冲区一半填满，全部填满，和IDLE接受完成三个地方，
// 这些触发器会修改Flag在主循环里面通过检查flag再读入缓冲。
// 我们设置成两个主循环的大小不会丢失数据。
#define serial_buffer_size 300
// 串口命令长度设置
#define cmd_length_limit 40


// LED
#define LED1_GPIO GPIOA
#define LED1_GPIO_PIN GPIO_PIN_8
#define LED2_GPIO GPIOC
#define LED2_GPIO_PIN GPIO_PIN_9

/* 以上是设置项目 Configurations End */

#endif