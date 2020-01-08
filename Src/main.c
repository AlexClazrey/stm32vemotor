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
#include <stdarg.h>
#include "./Dspin/dspin.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void change_speed(uint32_t cur, uint32_t target, uint32_t time,
		dSPIN_Direction_TypeDef direction);
void cycle_tick_sleep_to(uint32_t ms);
void cycle_tick_start();
int log_uart(char* cstr);
int log_uart_raw(uint8_t* data, uint16_t len);
void motor_run(int count);
void cmd_input(int count);
void cmd_run(char* cmd);
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
	/* USER CODE BEGIN 2 */
	while (L6470_BUSY1())
		;
	L6470_Configuration1();
	while (L6470_BUSY1())
		;

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	uint32_t count = (uint32_t) -1;
	uint32_t countintv = 10; // 10ms every cycle
	uint32_t countlim = 100000; // 1000s one big cycle
	while (1) {
		cycle_tick_start();
		// cycle count
		count++;
		if (count == countlim) {
			count = 0;
		}
		// cmd store
		cmd_input(count);
		// motor
		motor_run(count);


		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		cycle_tick_sleep_to(countintv);
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
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
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

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | GPIO_PIN_8, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);

	/*Configure GPIO pins : PC1 PC2 */
	GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PA4 PA8 */
	GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PC9 */
	GPIO_InitStruct.Pin = GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
int imax(int a, int b) {
	return a > b ? a : b;
}

int log_uart_raw(uint8_t* data, uint16_t len) {
	// 115200 Baud Rate ~= 80000 bps = 10KBps = 10 Byte / ms
	return HAL_UART_Transmit(&huart1, data, len, imax(len / 8, 5));
}

int log_uart(char* cstr) {
	return log_uart_raw((uint8_t*) cstr, strlen(cstr));
}

char logbuf[4096];
// output limit is 4095 char.
int log_uart_f(const char* format, ...) {
	va_list args;
	va_start(args, format);
	vsnprintf(logbuf, 4095, format, args);
	int res = log_uart(logbuf);
	va_end(args);
	return res;
}

uint32_t tickstart = 0;
void cycle_tick_start() {
	tickstart = HAL_GetTick();
}

void cycle_tick_sleep_to(uint32_t ms) {
	uint32_t elapsed = HAL_GetTick() - tickstart;
	if (ms < elapsed) {
		log_uart_f("[Warning] Cycle Time Exceeded, time used: %d, target: %d.", elapsed, ms);
		return;
	}
	while (ms > (HAL_GetTick() - tickstart))
		;
}

uint8_t cmdbuf[200] = { 0 };
uint32_t cmdlen = 0;
uint32_t cmdlim = 150; // command line length limit

// Read UART input if char is available.
// LF triggers cmd_run function.
void cmd_input(int count) {
	while (HAL_UART_Receive(&huart1, cmdbuf + cmdlen, 1, 0) == HAL_OK) {
		char last = (char) cmdbuf[cmdlen];
		cmdlen++;
		if (last == '\n') {
			// overwrite last LF char.
			cmdbuf[cmdlen - 1] = 0;
			cmd_run((char*) cmdbuf);
			cmdlen = 0;
		} else if (last == '\r') {
			// omit CR
			cmdlen--;
		} else if (last == '\b' || last == 0x7f) {
			// BS or DEL char, some terminal may send DEL instead of BS.
			cmdlen -= 2;
			if(cmdlen < 0) {
				cmdlen = 0;
			}
		}
		// if limit is reached.
		if (cmdlen >= cmdlim) {
			cmdbuf[cmdlen] = 0;
			cmd_run((char*) cmdbuf);
			cmdlen = 0;
		}
	}
}

void cmd_run(char* cmd) {
	// debug
	log_uart_f("Get Cmd: %s\n", cmd);
	// motor command
	if(strncmp(cmd, "motor run ", 10) == 0) {
		// cmd starts with "motor "
		// speed 3000 ~ 40000
		int speed, dir;
		int minsp = 3000, maxsp = 40000;
		sscanf(cmd, "motor run %d %d", &dir, &speed);
		if(speed < minsp || speed > maxsp) {
			log_uart_f("[Error] Speed not in range: %d, range is %d ~ %d.\n", speed, minsp, maxsp);
			return;
		}
		if(dir != 0 && dir != 1) {
			log_uart("[Error] Dir should be 0 or 1. \n");
			return;
		}
		dSPIN_Run(dir == 0 ? FWD : REV, speed);
	} else if(strncmp(cmd, "motor stop", 10) == 0) {
		dSPIN_Soft_Stop();
	} else {
		log_uart_f("Command not recognized: %s\n", cmd);
	}
}

int motor_is_init = 0;
void motor_run(int count) {
	if(!motor_is_init) {
		log_uart("Motor start\n");
		motor_is_init = 1;
		dSPIN_Run(REV, 1000);
	}
}

void change_speed(uint32_t cur, uint32_t target, uint32_t time,
		dSPIN_Direction_TypeDef direction) {
	while (L6470_BUSY1())
		;
	uint32_t acc = (target - cur) / time;
	// TODO ACC or DEC is not implemented in dspin.c
	if (acc > 0) {
		dSPIN_Set_Param(dSPIN_ACC, acc);
	} else {
		dSPIN_Set_Param(dSPIN_DEC, -acc);
	}
	while (L6470_BUSY1())
		;
	dSPIN_Run(direction, target);
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

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
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
