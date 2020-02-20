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
#include "Dspin/dspin.h"
#include "config.h"
#include "util.h"
#include "cycletick.h"
#include "log_uart.h"
#include "lm.h"
#include "can_io.h"
#include "command.h"
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
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

/* program variables */

// led
int led1_blink = 0;
int led2_blink = 0;

// cn1 press
int cn1_indicator = 0;
int lm_home_set = 0; // 在CN1按下的期间有没有校准过HOME位置

// motor
struct lm_handle lmh;
struct lm_handle *plmh = &lmh;

// CAN receive command
struct lm_cmd lmcan = { 0 };
// can buffer
char canbuf[8] = { 0 };
volatile int canlen = 0;

// main cycle
static uint32_t main_cycle_count = 0;

#define DMA_INPUT_SIZE (UART_INPUT_BUF_SIZE-1)

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CAN_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void detect_cn1();
void can_rx_int(uint8_t *data, uint8_t len);
void led1_flip();
void led2_flip();
void txcplt_report();
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
	MX_DMA_Init();
	MX_SPI1_Init();
	MX_USART1_UART_Init();
	MX_CAN_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */
	logu_init(&huart1, LOGU_DMA);
	logu_setlevel(LOGU_DEBUG);
//	logu_setlevel(LOGU_TRACE);
	// motor init
	L6470_Configuration1();
	lm_init(plmh);
	can_init();
	can_set_listener(can_rx_int);

	// 初始化的过程
	// 目前有向内移动直到 CN1 被按下这一步。
	logu_s(LOGU_DEBUG, "Start initializing.");
	logu_s(LOGU_DEBUG, "Moving to initial position.");
	lm_append_newcmd(plmh, lm_cmd_speed, 10000, 1);
	if (lm_commit(plmh)) {
		uint32_t init_tick = HAL_GetTick();
		while (1) {
			detect_cn1();
			if (cn1_indicator) {
				logu_s(LOGU_DEBUG, "Moving finished");
				lm_append_newcmd(plmh, lm_cmd_stop, 0, 0);
				lm_commit(plmh);
				break;
			} else if (HAL_GetTick() - init_tick > 10000) {
				logu_s(LOGU_ERROR, "Moving timeout.");
				lm_append_newcmd(plmh, lm_cmd_stop, 0, 0);
				lm_commit(plmh);
				break;
			}
		}
	} else {
		logu_s(LOGU_ERROR, "Reset position failed.");
	}
	logu_s(LOGU_DEBUG, "Initialize finished.");

	// input init
	/* Enable IDLE Interrupt with DMA circular reading */
	// Must clear state before enable idle or it will lost in a dead loop
	huart1.Instance->SR;
	huart1.Instance->DR;
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

	HAL_UART_Receive_DMA(&huart1, (uint8_t*) inputbuf_get(), DMA_INPUT_SIZE);

	// turn off LED2 after init
	LED2_GPIO->ODR |= LED2_GPIO_PIN;

	logu_s(LOGU_DEBUG, "Start listening.");

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		cycle_tick_start();
		if (led1_blink) {
			if ((main_cycle_count & 0x3f) == 0) { // 64 cycle
				led1_flip();
			}
		}
		if (led2_blink) {
			if ((main_cycle_count & 0x2f) == 0) {
				led2_flip();
			}
		}

		// detect switch on CN1
		detect_cn1();

		// can cache read
		if (canbuf_read(&lmcan, canbuf, canlen) == 1) {
			lm_append_cmd(plmh, (struct lm_cmd*) &lmcan);
		}
		canlen = 0;

		// read input buffer
		inputbuf_read(plmh);

		// cycle test 放在输入的位置后面
		mcycle_cmd(plmh, main_cycle_count);

		// 覆盖指令的操作都要确保这个指令之后确实用不到，不需要在下个周期重新触发。
		struct lm_cmd *lcf = lm_first(plmh);
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
				logu_s(LOGU_INFO, "Set home");
				lcf->type = lm_cmd_set_home;
			}
			if ((lcf->type == lm_cmd_speed && lcf->dir_hard == 1) || (lmmod.state == lm_state_speed && lmmod.dir == 1)
					|| (lcf->type == lm_cmd_pos && lcf->pos_speed > 0)
					|| (lmmod.state == lm_state_pos && lmmod.pos > 0)) {
				logu_s(LOGU_WARN, "Stop motor due to CN1.");
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
		// report
		txcplt_report();
		// cycle count
		main_cycle_count++;
		if (main_cycle_count == COUNT_LIMIT) {
			main_cycle_count = 0;
		}
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
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel4_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
	/* DMA1_Channel5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, L6470CS_Pin | GPIO_PIN_8, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);

	/*Configure GPIO pins : L6470Flag_Pin L6470Busy_Pin */
	GPIO_InitStruct.Pin = L6470Flag_Pin | L6470Busy_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : SW2_Pin */
	GPIO_InitStruct.Pin = SW2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(SW2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : L6470CS_Pin PA8 */
	GPIO_InitStruct.Pin = L6470CS_Pin | GPIO_PIN_8;
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
// ----------- Interrupts
// DMA Signals
static uint32_t txcpltcount = 0;
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == logu_getport()) {
		txcpltcount++;
		logu_dma_txcplt_callback();
	}
}
void txcplt_report() {
	if (maincyclecount() % 1000 == 0) {
		logu_f(LOGU_DEBUG, "log dma report: called %lu times in last 1000 cycles.", txcpltcount);
		txcpltcount = 0;
	}
}
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart1) {
		logu_s(LOGU_TRACE, "uart input buffer cycle half complete");
		inputbuf_setend(DMA_INPUT_SIZE / 2); // half complete is copy count / 2.
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart1) {
		logu_s(LOGU_TRACE, "uart input buffer cycle complete");
		inputbuf_setend(0);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	led2_blink = 1;
}

// Interrupt Routine For Serial IDLE
static uint32_t idlecount = 0;
void cmd_serial_int(UART_HandleTypeDef *huart) {
	if (huart == &huart1) {
		// error was handled by HAL, only check IDLE here.
		if (__HAL_UART_GET_FLAG(huart, USART_SR_IDLE)) {
			__HAL_UART_CLEAR_FLAG(huart, USART_SR_IDLE);
			logu_f(LOGU_TRACE, "uart input idle %lu", idlecount++);
			HAL_UART_DMAPause(huart);
			uint32_t dmacnt = __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
			inputbuf_setend(DMA_INPUT_SIZE - dmacnt);
			HAL_UART_DMAResume(huart);
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
				logu_s(LOGU_DEBUG, "CN1 pressed.");
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

int cn1_pressed() {
	return cn1_indicator;
}

// CAN read
void can_rx_int(uint8_t *data, uint8_t len) {
	// multiple CAN commands in one main cycle will drop and blink led2
	if (canlen == 0) {
		memcpy(canbuf, data, len);
		canlen = len;
	} else {
		led2_blink = 1;
	}
}

// LED
void led1_flip() {
	LED1_GPIO->ODR ^= LED1_GPIO_PIN;
}
void led2_flip() {
	LED2_GPIO->ODR ^= LED2_GPIO_PIN;
}

uint32_t maincyclecount() {
	return main_cycle_count;
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
	logu_f(LOGU_ERROR, "Assert failed: %s line %lu.", file, line);
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
