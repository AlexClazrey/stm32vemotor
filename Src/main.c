/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "./Dspin/dspin.h"
#include "util.h"
#include "log_uart.h"
#include "lm.h"
#include "can_io.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USE_FULL_ASSERT
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* 以下是设置项目 Configurations Start */

// 这个是机器编号，只能是一个字节。
uint8_t machine_id = 124;

// 移动到比例位置的时候使用的范围。
const int lm_limit_out = -110000;
const int lm_limit_in = -5000;

// 电机循环测试
const int lm_cycle_out = -10000;
const int lm_cycle_in = -5000;
// 1的时候只有跑完一整个测试循环，点击在最里面的位置才会暂停一会儿，
// 0的时候在测试里的每一步都会暂停一会儿。
int lm_cycle_pause_at_full_cycle = 1;

// 串口
// 考虑到 115200 一秒钟顶多10KB，那么一个主循环的约10ms不会超过1KB
#define serial_buffer_size 1000

// 命令长度设置
#define cmd_length_limit 100

// CAN 命令参数
const uint8_t CAN_CMD_VER = 2;
const uint8_t CAN_CMD_LM = 1;
const uint8_t CAN_CMD_STRING = 10;

// 主循环时间设置
const uint32_t COUNT_INTV = 10; // 每次主循环使用的毫秒数
const uint32_t COUNT_LIMIT = 1000; // COUNT_LIMIT * COUNT_INTV / 1000 得到一个大循环使用的秒数，现在是100s。

// LED
#define LED1_GPIO GPIOA
#define LED1_GPIO_PIN GPIO_PIN_8
#define LED2_GPIO GPIOC
#define LED2_GPIO_PIN GPIO_PIN_9

/* 以上是设置项目 Configurations End */

/* program variables */

// led1
int led1_blink = 0;

// cn1 press
int cn1_indicator = 0;
int lm_home_set = 0; // 在CN1按下的期间有没有校准过HOME位置

// motor
struct lm_handle lmh;
struct lm_handle *plmh = &lmh;

// motor cycle test
int lm_cycle = 0; // 循环测试开启指示
uint32_t lm_cycle_pause_count = 0; // 暂停的等待计数
uint32_t lm_cycle_speed_count = 0; // 在过热无反应时候的等待计数

// CAN receive command
volatile struct lm_cmd lmcan = { 0 };
volatile int lmcan_cached = 0;

// Serial input command
uint8_t inputbuf[serial_buffer_size] = { 0 }; // 这个不需要 volatile 因为在读取这段的过程里读取的内容不会被更新改变，所以可以缓存。
volatile int inputstart = 0; // 这个在中断程序片段看来没有修改过的变量实际上会被主循环修改，这种情况也要volatile
volatile int inputend = 0;

// cmd parse buffer
char cmdbuf[cmd_length_limit + 1] = { 0 };

// tick value when a main cycle starts
uint32_t tickstart = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CAN_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void mcycle_append_cmd(struct lm_handle* plmh, uint32_t count);
void cycle_tick_start();
void cycle_tick_sleep_to(uint32_t ms);
uint32_t cycle_tick_now();
void inputbuf_read(struct lm_handle* plmh);
void detect_cn1();
void can_cmd_run(uint8_t* data, uint8_t len);
void led1_flip();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_SPI1_Init();
	MX_USART1_UART_Init();
	MX_CAN_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */
	L6470_Configuration1();
	log_set_port(&huart1);
	can_init();
	can_set_listener(can_cmd_run);
	lm_init(plmh);

	// 初始化的过程
	// 目前有向内移动直到 CN1 被按下这一步。
	log_uart(LOGDEBUG, "Start initializing.");
	log_uart(LOGDEBUG, "Moving to initial position.");
	lm_append_newcmd(plmh, lm_cmd_speed, 10000, 1);
	if (lm_commit(plmh)) {
		uint32_t init_tick = HAL_GetTick();
		while (1) {
			detect_cn1();
			if (cn1_indicator) {
				log_uart(LOGDEBUG, "Moving finished");
				lm_append_newcmd(plmh, lm_cmd_reset, 0, 0);
				lm_commit(plmh);
				break;
			} else if (HAL_GetTick() - init_tick > 10000) {
				log_uart(LOGERROR, "Moving timeout.");
				break;
			}
		}
	} else {
		log_uart(LOGERROR, "Reset position failed.");
	}
	log_uart(LOGDEBUG, "Initialize finished.");

	// turn off LED2 after init
	LED2_GPIO->ODR |= LED2_GPIO_PIN;

	log_uart(LOGDEBUG, "Start listening.");


	// 开中断，这一段参照 xxxxhal_uart.c 里面的说明
	// 这个要配合自动生成的 HAL_NVIC_EnableIRQ 一起用才有效果
	/* Enable the UART Data Register not empty Interrupt */
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
	/* Enable the UART Parity Error Interrupt */
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_PE);
	/* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_ERR);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	uint32_t count = (uint32_t) -1;
	while (1) {
		cycle_tick_start();
		// cycle count
		count++;
		if (count == COUNT_LIMIT) {
			count = 0;
		}

		if (led1_blink) {
			if ((count & 0x3f) == 0) { // 64 cycle
				led1_flip();
			}
		}

		// detect switch on CN1
		detect_cn1();

		// can cache read
		if (lmcan_cached) {
			lm_append_cmd(plmh, (struct lm_cmd*) &lmcan);
			lmcan_cached = 0;
		}

		// read input buffer
		inputbuf_read(plmh);

		// cycle test 放在输入的位置后面
		if (lm_cycle) {
			mcycle_append_cmd(plmh, count);
		} else {
			__NOP();
		}

		// 覆盖指令的操作都要确保这个指令之后确实用不到，不需要在下个周期重新触发。
		struct lm_cmd* lcf = lm_first(plmh);
		if (lcf == NULL) {
			// 如果命令队列是空的话，那么这里制作一个命令。
			lm_append_newcmd(plmh, lm_cmd_empty, 0, 0);
			lcf = lm_first(plmh);
		}
		struct lm_model lmmod = plmh->mod; // make a copy of volatile model

		// if cn1 is pressed only stop 1 - FWD - close direction
		// TODO add pos detection after implementing read pos function
		if (cn1_indicator) {
			// cn1 按下的时候重置 home，这个只能在按下期间执行，
			// 因为在 pos 状态下重置 home 会发生惨剧，会把现在高速移动的某个位置标记成HOME，
			// 导致在向外移动的过程里面如果按下HOME那么就会冲出范围。
			// 在向内移动的过程中按下HOME会在下一次向外运动的时候冲出范围。
			// 所以以我关闭了pos运动模式里面的误差矫正。
			// 只有匀速运动的模式里面按下cn1会矫正位置。
			// 如果要解决越来越向内的误差的话，在使用POS命令的时候，在收回的时候留一些空档。
			if (!lm_home_set && lmmod.state != lm_state_pos) {
				log_uart(LOGINFO, "Set home");
				lcf->type = lm_cmd_set_home;
			}
			// 如果使用 lmcmd_immed 的话，下面的判断有可能在错误的时间执行。
			// 所以就好好用普通指令。
			if ((lcf->type == lm_cmd_speed && lcf->dir_hard == 1) || (lmmod.state == lm_state_speed && lmmod.dir == 1)
					|| (lcf->type == lm_cmd_pos && lcf->pos_speed > 0)
					|| (lmmod.state == lm_state_pos && lmmod.pos > 0)) {
				log_uart(LOGWARN, "Stop motor due to CN1.");
				lcf->type = lm_cmd_stop;
				lcf->dir_hard = 1;
			}
		} else {
			lm_home_set = 0;
		}

		// TODO 如果过冲进入了大电流保护状态那么重置 L6470
		//
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		if (lcf->type == lm_cmd_set_home) {
			lm_home_set = 1;
		}
		lm_commit(plmh);
		cycle_tick_sleep_to(COUNT_INTV);
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief CAN Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN_Init(void) {

	/* USER CODE BEGIN CAN_Init 0 */

	/* USER CODE END CAN_Init 0 */

	/* USER CODE BEGIN CAN_Init 1 */

	/* USER CODE END CAN_Init 1 */
	hcan.Instance = CAN1;
	hcan.Init.Prescaler = 16;
	hcan.Init.Mode = CAN_MODE_NORMAL;
	hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan.Init.TimeSeg1 = CAN_BS1_4TQ;
	hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
	hcan.Init.TimeTriggeredMode = DISABLE;
	hcan.Init.AutoBusOff = DISABLE;
	hcan.Init.AutoWakeUp = DISABLE;
	hcan.Init.AutoRetransmission = DISABLE;
	hcan.Init.ReceiveFifoLocked = DISABLE;
	hcan.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN CAN_Init 2 */

	/* USER CODE END CAN_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
	hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | GPIO_PIN_8, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);

	/*Configure GPIO pins : PC1 PC2 */
	GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : SW2_Pin */
	GPIO_InitStruct.Pin = SW2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(SW2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PA4 PA8 */
	GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : CN1_Pin */
	GPIO_InitStruct.Pin = CN1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(CN1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PC9 */
	GPIO_InitStruct.Pin = GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 4 */

#define true 1
#define false 0

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
		log_uart_f(LOGWARN, "Cycle Time Exceeded, time used: %d, target: %d.", elapsed, ms);
		return;
	}
	while (cycle_tick_now() < ms)
		;
}

// ----------- Interrupts
// Interrupt Routine For Serial Input
volatile uint32_t hole;
void cmd_serial_int(UART_HandleTypeDef *huart) {
	// check error
	uint32_t isr = huart->Instance->SR;
	uint32_t error = (isr & (uint32_t) (USART_SR_PE | USART_SR_FE | USART_SR_ORE | USART_SR_NE));
	char last;
	if (error) {
		// 异常发生的时候，要不就是因为垃圾数据，要不就是因为在处理读入的时候收到了下一位数据，
		// 两种情况都应该抛弃数据
		hole = huart->Instance->DR;
	}
	if (isr & (uint32_t) USART_SR_RXNE) {
		if (inputend == inputstart - 1 || (inputend == serial_buffer_size - 1 && inputstart == 0)) {
			// if buffer full, drop data
			hole = huart->Instance->DR;
		} else {
			// if buffer still has space
			inputbuf[inputend++] = last = huart->Instance->DR;

			if (last == '\b' || last == 0x7f) {
				// BS or DEL char, some terminal may send DEL instead of BS.
				inputend -= 2;

				if (inputend == inputstart - 1)
					inputend = inputstart;
				else if (inputend < 0)
					inputend += serial_buffer_size;
			} else if (last == '\0') {
				--inputend;
			}

			if (inputend == serial_buffer_size)
				inputend = 0;
		}
	}
}

// ------------- Input Buffer Process
void input_feedback(int success);
enum inputcmdtype inputbuf_read_one_cmd(struct lm_cmd *out_pcmd, uint8_t *out_receiver);
int cmd_copy(char* dest, char* buf, int start, int end, int bufsize, int cmdsize);
enum inputcmdtype cmd_parse(char* cmd, struct lm_cmd* out_store, uint8_t *out_recevier);
HAL_StatusTypeDef can_cmd_send(struct lm_cmd* cmd, uint8_t receiver_id);
HAL_StatusTypeDef can_send_log(HAL_StatusTypeDef send_res);

enum inputcmdtype {
	input_error, // 输入出错时候的信号
	input_empty, // 没有输入到行尾的时候的返回信号
	input_next,  // 连续两个行尾，比如说输入了一个空行的时候的信号
	input_lmcmd, // 输入了一个本机的电机命令时候的信号
	input_settings, // 输入了一个本机设置命令时候的信号
	input_can, // 输入了CAN命令时候的信号。
};

void inputbuf_read(struct lm_handle* plmh) {
	// 这个函数试把解析InputBuffer的工作移动到主循环里面，这样可以面对大批量的命令汇入
	// 首先先向后寻找\r\n或者\0的位置，复制到cmdbuf里面，然后清理cmdbuf，然后解析。
	// 为了能在一次主循环里面处理多条命令。
	// 需要创建不止一个命令缓存而是很多个命令缓存池
	// 如果命令缓存池还有一个空位那就尝试读取
	// 因为不是自己的命令会沿着CAN发送出去不会占用空位
	// 如果是自己的命令那么占用一个位置
	// 如过缓存池被用满了那么停止这里的Read Input同时应该发出一个警告。
	// 发送很多个CAN指令也可能消耗很多时间，所以这里还需要在Parse一个新的命令之前注意检查时间，
	// 如果时间不多了那么就留到下一个循环处理。
	// 我也需要log一下处理一条命令需要用多少时间。
	// copy to cmd buffer and parse
	//
	while (cycle_tick_now() < 8) {
		if (!lm_hasspace(plmh)) {
			log_uart(LOGWARN, "Stop reading input due to full cmd queue");
			return;
		}
		struct lm_cmd cmd = { 0 };
		uint8_t receiver = 0;
		enum inputcmdtype type = inputbuf_read_one_cmd(&cmd, &receiver);

		if (type == input_error) {
			// Error details was printed in parse function
			input_feedback(false);
		} else if (type == input_empty) {
			// 读到empty表示输入缓冲区已经读完了。
			break;
		} else if (type == input_lmcmd) {
			// 如果是本机的命令，那么加入cmd queue。
			lm_append_cmd(plmh, &cmd);
			input_feedback(true);
		} else if (type == input_settings) {
			input_feedback(true);
		} else if (type == input_can) {
			HAL_StatusTypeDef hs = can_cmd_send(&cmd, receiver);
			can_send_log(hs);
			input_feedback(hs == HAL_OK);
		} else if (type == input_next) {
			// do nothing here
		} else {
			log_uart(LOGERROR, "Unknown error [01]");
		}
	}

}

enum inputcmdtype inputbuf_read_one_cmd(struct lm_cmd *out_pcmd, uint8_t *out_receiver) {
	uint32_t cursor = inputstart;
	int triggered = 0;
	while (cursor != (uint32_t) inputend) {
		// check char here
		char ch = inputbuf[cursor];
		if (ch == '\0' || ch == '\r' || ch == '\n') {
			triggered = 1;
			break;
		}
		cursor++;
		if (cursor == serial_buffer_size) {
			cursor = 0;
		}
	}
	if (!triggered) {
		return input_empty;
	}

	if (cursor == (uint32_t) inputstart) {
		log_uart_f(LOGDEBUG, "Yet another line ending.");
		inputstart = addu(cursor, 1, serial_buffer_size);
		return input_next; // 现在如果 \r\n 就会触发一次 next
	}

	int res = cmd_copy(cmdbuf, (char*) inputbuf, inputstart, cursor, serial_buffer_size, cmd_length_limit);
	// 这个时候cursor指向的是 \r\n\0 不可能是 inputend 所以要再加一
	inputstart = addu(cursor, 1, serial_buffer_size);

	if (res) {
		return cmd_parse(cmdbuf, out_pcmd, out_receiver);
	} else {
		return input_error;
	}
}

void input_feedback(int success) {
	if (success) {
		log_uart_raw((uint8_t*) "<OK>\r\n", 6);
	} else {
		log_uart_raw((uint8_t*) "<FAIL>\r\n", 8);
	}
}

HAL_StatusTypeDef can_send_log(HAL_StatusTypeDef send_res) {
	if (send_res == HAL_OK) {
		log_uart_f(LOGDEBUG, "CAN Sent.");
	} else {
		log_uart_f(LOGERROR, "CAN Send failed.");
	}
	return send_res;
}

// --------- Command
int cmd_parse_body(char* cmd, struct lm_cmd *store);
int read_mr_args(char* cmd, uint32_t* out_speed, uint32_t* out_dir);

/* Command format is:
 * [#<id>](mr|ms|mp|mpp|mreset|mhome|mcycle|mid)[ <args>]
 */

// return 1 is good, 0 is error.
int cmd_copy(char* dest, char* buf, int start, int end, int bufsize, int cmdsize) {
	int len = cycarrcpy(dest, buf, start, end, bufsize, cmdsize);
	// 这里要记得 cstr 末尾的 NUL 字符
	if (len != -1) {
		dest[len] = '\0';
		return 1;
	} else {
		log_uart_f(LOGERROR, "Input longer than cmd length limit.");
		return 0;
	}
}

enum inputcmdtype cmd_parse(char* cmd, struct lm_cmd* out_store, uint8_t *out_receiver) {
	uint16_t receiver = (uint16_t) -1;

	log_uart_f(LOGDEBUG, "Parse: %s", cmd);

	// 为了兼容之前的格式
	if (strncmp(cmd, "sig ", 4) == 0) {
		cmd += 4;
	}

	// if command has an receiver
	if (cmd[0] == '#') {
		size_t scanlen;
		if (sscanf(++cmd, "%hu%n", &receiver, &scanlen) != 1 || receiver > 255) {
			log_uart(LOGERROR, "Cmd id read error.");
			return 1;
		}
		*out_receiver = (uint8_t) receiver;
		if (cmd[scanlen] == ' ') {
			// 兼容ID后面有或者没有空格两种情况
			cmd++;
		}
		cmd += scanlen;
	}

	// parse command body
	int res = cmd_parse_body(cmd, out_store);

	if (res == 1) {
		return input_error;
	} else if (res == 2) {
		return input_settings;
	} else if (res == 0) {
		if (receiver == machine_id || receiver == (uint16_t) -1) {
			return input_lmcmd;
		} else {
			return input_can;
		}
	} else {
		log_uart_f(LOGERROR, "Unknown error [02]");
		return input_error;
	}
}

// return 0: lm_cmd command, command which actually modifies lm_cmd* store.
// return 1: parse failed
// return 2: settings command
int cmd_parse_body(char* cmd, struct lm_cmd *store) {
	// any input will turn off motor cycle test
	if (lm_cycle) {
		log_uart_f(LOGINFO, "Cycle off.");
		lm_cycle = 0;
	}
	if (cmd[0] == 'm') {
		if (strncmp(cmd + 1, "cycle", 5) == 0) {
			// TODO 把循环测试也做成能用CAN发送的命令。
			// TODO settings command 包括 mid 和 mcycle 都没有储存机制
			lm_cycle = !lm_cycle;
			if (lm_cycle) {
				// cycle kick start action, move to cycle_in position
				store->type = lm_cmd_pos;
				store->pos_speed = lm_cycle_in;
				return 0;
			}
			return 2;
		} else if (strncmp(cmd + 1, "home", 4) == 0) {
			store->type = lm_cmd_set_home;
		} else if (strncmp(cmd + 1, "reset", 5) == 0) {
			store->type = lm_cmd_reset;
		} else if (strncmp(cmd + 1, "id ", 3) == 0) {
			uint16_t id;
			if (sscanf(cmd + 4, "%hu", &id) == 1 && id < 256) {
				machine_id = id;
				return 2;
			} else {
				log_uart(LOGERROR, "Read Machine Id Error, should between 0 and 255.");
				return 1;
			}
		} else if (cmd[1] == 'r' && cmd[2] == ' ') {
			// 这里一定要注意检验 cmd[2] 是空格，不然可能是NUL字符，就是一定要连续检验，想不到还有这个隐患。
			uint32_t pos, dir;
			if (read_mr_args(cmd + 3, &pos, &dir) != 2) {
				log_uart(LOGERROR, "Read mr command failed.");
				return 1;
			}
			store->pos_speed = pos;
			store->dir_hard = dir;
			store->type = lm_cmd_speed;
		} else if (cmd[1] == 's') {
			store->type = lm_cmd_stop;
		} else if (cmd[1] == 'p') {
			if (cmd[2] == ' ') {
				if (sscanf(cmd + 3, "%ld", &(store->pos_speed)) != 1) {
					log_uart(LOGERROR, "Read mp command failed.");
					return 1;
				}
			} else if (cmd[2] == 'p' && cmd[3] == ' ') {
				uint16_t percent;
				if (sscanf(cmd + 4, "%hu", &percent) != 1) {
					log_uart(LOGERROR, "Read mpp percent failed.");
					return 1;
				}
				if (percent > 100) {
					log_uart(LOGERROR, "Mpp percent should between 0 and 100.");
					return 1;
				}
				store->pos_speed = (int) ((double) percent / 100 * (lm_limit_out - lm_limit_in) + lm_limit_in);
			}
			store->type = lm_cmd_pos;
		} else {
			log_uart(LOGERROR, "Command parse failed");
			return 1;
		}
	} else {
		log_uart(LOGERROR, "Command parse failed");
		return 1;
	}
	return 0;
}

int read_mr_args(char* cmd, uint32_t* out_speed, uint32_t* out_dir) {
	uint32_t dir, speed;
	// speed 1 ~ 30
	// 太高的速度在冲击时会引发L6470过流关闭，需要 L6470 Reset 指令重新开启
	uint32_t minsp = 1, maxsp = 30;
	if (sscanf(cmd, "%lu %lu", &dir, &speed) != 2) {
		log_uart(LOGERROR, "Parse args failed.");
		return 0;
	}
	if (speed < minsp || speed > maxsp) {
		log_uart_f(LOGERROR, "Speed not in range: %d, range is %d ~ %d.", speed, minsp, maxsp);
		return 0;
	}
	if (dir != 0 && dir != 1) {
		log_uart(LOGERROR, "Dir should be 0 or 1.");
		return 0;
	}
	*out_speed = speed * 1000;
	*out_dir = dir;
	return 2;
}

// ----------
// 设置电机循环测试的时候使用的命令。
void mcycle_append_cmd(struct lm_handle *plmh, uint32_t count) {
	// TODO 芯片如果出现移动到一半就认为自己移动完成的情况，是奇怪BUG的开始，这是因为芯片过热了，这个时候也需要考虑处理的对策。
	if (L6470_BUSY1()) {
		// 在运转到某个位置的途中会出现BUSY
		// 其他时候都不会有BUSY
		// TODO 因为现在POS状态下的校准HOME是关闭的（在之后的注释里），所以一旦移动向内的时候误差向内没有一个HOME校准，后面可能向内的误差累积都没有校准。
		// 这就会引发问题。
		lm_cycle_pause_count = count;
	} else if (lm_cycle_pause_at_full_cycle || diffu(count, lm_cycle_pause_count, COUNT_LIMIT) > 100) {
		struct lm_model lmmod = plmh->mod;
		// 这个if对应如果设置在每一步暂停一会儿，那么等待一段count计数
		if ((lmmod.state == lm_state_pos && lmmod.pos == lm_cycle_in) || lmmod.state == lm_state_stop
				|| lmmod.state == lm_state_speed) {
			// state pos + lm_cycle_in 是运转到了最里面刚停下的状态
			// state stop 是匀速模式碰到了CN1之后的结果
			// state speed 是进入了匀速模式
			if (cn1_indicator) {
				// 当移动到最内侧CN1被按下。
				if (!lm_cycle_pause_at_full_cycle || diffu(count, lm_cycle_pause_count, COUNT_LIMIT) > 100) {
					// 向外移动
					log_uart(LOGINFO, "Move forward");
					lm_append_newcmd(plmh, lm_cmd_pos, lm_cycle_out, 0);
				}
			} else if (lmmod.state != lm_state_speed) {
				// 当POS移动完成，没有按下CN1的时候进入向内匀速运动的模式。
				lm_cycle_speed_count = count;
				log_uart(LOGINFO, "Finding Home");
				lm_append_newcmd(plmh, lm_cmd_speed, 10000, 1);
			} else {
				// 进入匀速模式但是没有到头的情况
				if (diffu(count, lm_cycle_speed_count, COUNT_LIMIT) > 200) {
					// 太长时间没有到头说明点击进入了过热保护，这个时候等待一段时间再下命令会好。
					lm_cycle_speed_count = count;
					log_uart(LOGWARN, "Finding Home Again");
					lm_append_newcmd(plmh, lm_cmd_speed, 10000, 1);
				}
			}
		} else if (lmmod.state == lm_state_pos && lmmod.pos == lm_cycle_out) {
			// 当移动到最外侧
			log_uart(LOGINFO, "Move back");
			lm_append_newcmd(plmh, lm_cmd_pos, lm_cycle_in, 0);
		}
	}
}

static int cn1_hold_count = 0;
void detect_cn1() {
	// if CN1 signal is low
	if (!(CN1_GPIO_Port->IDR & CN1_Pin)) {
		// avoid shake - hold at least 5 cycles to trigger
		if (cn1_hold_count >= 5) {
			if (!cn1_indicator) {
				log_uart(LOGDEBUG, "CN1 pressed.");
				cn1_indicator = 1;
			}
		} else {
			cn1_hold_count++;
		}
	} else {
		cn1_indicator = 0;
		cn1_hold_count = 0;
	}
}

// CAN send lm_cmd
HAL_StatusTypeDef can_cmd_send(struct lm_cmd* cmd, uint8_t receiver_id) {
	uint8_t msg[8];
	uint32_t pos = cmd->pos_speed;
	msg[0] = CAN_CMD_VER;
	msg[1] = CAN_CMD_LM;
	msg[2] = receiver_id;
	msg[3] = (uint8_t) (cmd->type);
	msg[4] = (uint8_t) (pos >> 16);
	msg[5] = (uint8_t) (pos >> 8);
	msg[6] = (uint8_t) (pos);
	msg[7] = (uint8_t) (cmd->dir_hard);
	return can_msg_add(msg, 8);
}

// when CAN receives data through interrupt
void can_cmd_run(uint8_t* data, uint8_t len) {
	// no lm_commit called here, program will commit in main cycle.
	// 考虑到中断可能发生在主循环的任何地方，这里不要直接更改主循环里面使用的变量。
	// 只能在主循环里面做一个JOIN信息的步骤，就算JOIN信息做在中断里面一个通知变量也只能在主循环里接受。
	// 这就像多线程一样要注意公共区域修改的时机。
	if (len >= 3) {
		uint8_t ver = data[0], cmd = data[1], id = data[2];
		if (id == machine_id) {
			if (ver > CAN_CMD_VER) {
				log_uart_f(LOGERROR, "CAN Message Version Higher than me.");
				return;
			}
			if (cmd == CAN_CMD_LM) {
				if (lmcan_cached) {
					log_uart_f(LOGWARN, "CAN Ignored: previous command is still cached.");
					return;
				}
				lmcan_cached = 1;
				// 按照 cmd_run 里面的编码方法重新组装消息
				lmcan.type = data[3];
				lmcan.pos_speed = (int32_t) ((data[4] << 16) + (data[5] << 8) + (data[6]));
				lmcan.dir_hard = data[7];
			} else if (cmd == CAN_CMD_STRING) {
				char buf[8] = { 0 };
				memcpy(buf, data + 2, len - 2);
				log_uart_f(LOGINFO, "CAN Received: %s", buf);
			}
		} else {
			log_uart_f(LOGDEBUG, "Ignore CAN Message with other's id #%hu", (uint16_t) id);
		}
	} else {
		log_uart_f(LOGWARN, "CAN Invalid Message: Too Short.");
	}
}

// LED
void led1_flip() {
	LED1_GPIO->ODR ^= LED1_GPIO_PIN;
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	led1_blink = 1;
	led1_flip();
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	log_uart_f(LOGERROR, "Assert failed: %s line %lu.", file, line);
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
