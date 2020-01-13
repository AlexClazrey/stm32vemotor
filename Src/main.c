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
#include "log_uart.h"
#include "lm.h"
#include "can_io.h"
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
CAN_HandleTypeDef hcan;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
int cn1_indicator = 0; // 这个变量告诉你cn1有没有被按
struct lm_model lmmod = { 0 };
struct lm_cmd lmcmd = { 0 };
//struct lm_cmd lmcmd_immed = { 0 };
// 不要用即时指令，这会打乱逻辑，
// 更改flag的一些命令想知道到底有没有成功用之后的检测。
int lm_cycle = 0; // 循环测试指示器
const int lm_cycle_out = -110000;
const int lm_cycle_in = -5000;
int lm_home_set = 0; // 在CN1按下的期间有没有校准过HOME位置

uint8_t cmdbuf[200] = { 0 };
int32_t cmdlen = 0; // must be signed here
uint32_t cmdlim = 150; // command line length limit

const uint8_t CAN_CMD_VER = 2;
const uint8_t CAN_CMD_LM = 1;
const uint8_t CAN_CMD_STRING = 10;

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
void cycle_tick_sleep_to(uint32_t ms);
void cycle_tick_start();
void cmd_input(int count);
void cmd_parse(char* cmd);
void detect_cn1();
void ds_acc_set(uint32_t cur, uint32_t target, uint32_t time_sec);
void ds_commit();
void can_cmd_run(uint8_t* data, uint8_t len);
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

	log_uart(LOGDEBUG, "Start initializing.");
	log_uart(LOGDEBUG, "Moving to initial position.");
	lmcmd.type = lm_cmd_speed;
	lmcmd.dir_hard = 1;
	lmcmd.pos_speed = 10000;
	if (lm_commit(&lmcmd, &lmmod) != 0) {
		log_uart(LOGERROR, "Reset position failed.");
	} else {
		uint32_t init_tick = HAL_GetTick();
		while (1) {
			detect_cn1();
			if (cn1_indicator) {
				log_uart(LOGDEBUG, "Moving finished");
				lmcmd.type = lm_cmd_reset;
				lm_commit(&lmcmd, &lmmod);
				break;
			} else if (HAL_GetTick() - init_tick > 30000) {
				log_uart(LOGERROR, "Moving timeout.");
				break;
			}
		}
	}
	log_uart(LOGDEBUG, "Initialize finished.");
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	log_uart(LOGDEBUG, "Start listening.");
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
		// TODO 其实读取 UART 输入应该做成中断模式的，直接写在循环里面是 Arduino 导致的坏习惯会限制很多
		// TODO 而且串口要比 CAN 不稳定得多，有可能会漏掉一些字节。
		// 这些稳定性的确认重发都没有做。
		cmd_input(count);
		// detect switch on CN1
		detect_cn1();


		// 这芯片也不是完美的，有时候移动到一半就认为自己移动完了这个很麻烦。
		if (lm_cycle) {
			if (L6470_BUSY1()) {
				// do nothing here
			} else if ((lmmod.state == lm_state_pos && lmmod.pos == lm_cycle_in)
					|| lmmod.state == lm_state_stop) {
				// state stop is for cn1 being pressed.
				if (cn1_indicator) {
					log_uart(LOGINFO, "Move forward");
					lmcmd.type = lm_cmd_pos;
					lmcmd.pos_speed = lm_cycle_out;
				} else {
					if (lmmod.state != lm_state_speed) {
						log_uart(LOGINFO, "Finding Home");
						lmcmd.type = lm_cmd_speed;
						lmcmd.pos_speed = 10000;
						lmcmd.dir_hard = 1;
					}
				}
			} else if (lmmod.state == lm_state_pos && lmmod.pos == lm_cycle_out) {
				log_uart(LOGINFO, "Move back");
				lmcmd.type = lm_cmd_pos;
				lmcmd.pos_speed = lm_cycle_in;
			}
		}

		// if cn1 is pressed only stop 1 - FWD - close direction
		// TODO add pos detection after implementing read pos function
		if (cn1_indicator) {
			// cn1 按下的时候重置 home，这个只能在按下期间执行一次
			// 在 pos 状态下重置 home 会发生惨剧。
			// 所以如果要解决越来越超内的误差的话，就不要回到0位置。
			if(!lm_home_set && lmmod.state != lm_state_pos) {
				log_uart(LOGINFO, "Set home");
				lmcmd.type = lm_cmd_set_home;
			}
			// 如果使用 lmcmd_immed 的话，下面的判断有可能在错误的时间执行。
			// 所以就好好用普通指令
			if ((lmcmd.type == lm_cmd_speed && lmcmd.dir_hard == 1)
					|| (lmmod.state == lm_state_speed && lmmod.dir == 1)
					|| (lmcmd.type == lm_cmd_pos && lmcmd.pos_speed > 0)
					|| (lmmod.state == lm_state_pos && lmmod.pos > 0)) {
				log_uart(LOGWARN,
						"CN1 is pressed, motor will not move backward.");
				lmcmd.type = lm_cmd_stop;
				lmcmd.dir_hard = 1;
			}
		} else {
			lm_home_set = 0;
		}


		// 如果过冲进入保护状态那么重置 L6470
		// TODO
		/* USER CODE END WHILE */
		/* USER CODE BEGIN 3 */
		if(lmcmd.type == lm_cmd_set_home) {
			lm_home_set = 1;
		}
		lm_commit(&lmcmd, &lmmod);
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
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
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
	/** Enables the Clock Security System
	 */
	HAL_RCC_EnableCSS();
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

// ---------- Time
void cycle_tick_start() {
	tickstart = HAL_GetTick();
}

void cycle_tick_sleep_to(uint32_t ms) {
	uint32_t elapsed = HAL_GetTick() - tickstart;
	if (ms < elapsed) {
		log_uart_f(LOGWARN, "Cycle Time Exceeded, time used: %d, target: %d.",
				elapsed, ms);
		return;
	}
	while (ms > (HAL_GetTick() - tickstart))
		;
}

// --------- Command
// Read UART input if char is available.
// LF or CR triggers cmd_run function.
void cmd_input(int count) {
	while (HAL_UART_Receive(&huart1, cmdbuf + cmdlen, 1, 0) == HAL_OK) {
		char last = (char) cmdbuf[cmdlen];
		cmdlen++;
		if (last == '\n' || last == '\r') {
			if (cmdlen > 0) {
				// overwrite last LF / CR char.
				cmdbuf[cmdlen - 1] = 0;
				cmd_parse((char*) cmdbuf);
				cmdlen = 0;
			}
		} else if (last == '\b' || last == 0x7f) {
			// BS or DEL char, some terminal may send DEL instead of BS.
			cmdlen -= 2;
			if (cmdlen < 0) {
				cmdlen = 0;
			}
		}
		// if limit is reached.
		if (cmdlen >= cmdlim) {
			cmdbuf[cmdlen] = 0;
			cmd_parse((char*) cmdbuf);
			cmdlen = 0;
		}
	}
}
static int read_mr_args(char* cmd, uint32_t* out_speed, uint32_t* out_dir);
static HAL_StatusTypeDef can_send_feedback(HAL_StatusTypeDef send_res);

/* Command format is:
 * [#<id> ](mr|ms|mp|mreset|mhome|mcycle)[ <args>]
 */
uint32_t my_cmdid = 123;
void cmd_parse(char* cmd) {
	log_uart_f(LOGDEBUG, "Run: %s\n", cmd);

	// if command has an id
	uint32_t cmdid = 0;
	if (cmd[0] == '#') {
		size_t scanlen;
		if (sscanf(cmd + 1, "%lu%n ", &cmdid, &scanlen) != 1) {
			log_uart(LOGERROR, "Cmd id read error.");
			return;
		}
		cmd += scanlen + 1;
	}

	// parse command body
	if (cmd[0] == 'm') {
		if (strncmp(cmd + 1, "cycle", 5) == 0) {
			lm_cycle = !lm_cycle;
			if(lm_cycle) {
				// kickstart
				lmcmd.type = lm_cmd_pos;
				lmcmd.pos_speed = lm_cycle_in;
			}
		} else {
			if(lm_cycle) {
				log_uart_f(LOGINFO, "Cycle off.");
				lm_cycle = 0;
			}
			if (strncmp(cmd + 1, "home", 4) == 0) {
				lmcmd.type = lm_cmd_set_home;
			} else if (strncmp(cmd + 1, "reset", 5) == 0) {
				lmcmd.type = lm_cmd_reset;
			} else if (cmd[1] == 'r') {
				uint32_t pos, dir;
				if (read_mr_args(cmd + 3, &pos, &dir) != 2) {
					log_uart(LOGERROR, "Read mr command failed.");
					return;
				}
				lmcmd.pos_speed = pos;
				lmcmd.dir_hard = dir;
				lmcmd.type = lm_cmd_speed;
			} else if (cmd[1] == 's') {
				lmcmd.type = lm_cmd_stop;
			} else if (cmd[1] == 'p') {
				if (sscanf(cmd + 3, "%ld", &(lmcmd.pos_speed)) != 1) {
					log_uart(LOGERROR, "Read mp command failed.");
					return;
				}
				lmcmd.type = lm_cmd_pos;
			} else {
				log_uart(LOGERROR, "Command parse failed");
				return;
			}
		}
	} else {
		log_uart(LOGERROR, "Command parse failed");
		return;
	}

	// check local or CAN
	if (cmdid == 0 || cmdid == my_cmdid) {
	} else {
		uint8_t msg[8];
		uint32_t pos = lmcmd.pos_speed;
		msg[0] = CAN_CMD_VER;
		msg[1] = CAN_CMD_LM;
		msg[2] = (uint8_t) (lmcmd.type);
		msg[3] = (uint8_t) (pos >> 24);
		msg[4] = (uint8_t) (pos >> 16);
		msg[5] = (uint8_t) (pos >> 8);
		msg[6] = (uint8_t) (pos);
		msg[7] = (uint8_t) (lmcmd.dir_hard);
		can_send_feedback(can_msg_add(msg, 8));
	}
}

static int read_mr_args(char* cmd, uint32_t* out_speed, uint32_t* out_dir) {
	uint32_t dir, speed;
	// speed 1 ~ 30
	// 太高的速度在冲击时会引发L6470过流关闭
	uint32_t minsp = 1, maxsp = 30;
	if (sscanf(cmd, "%lu %lu", &dir, &speed) != 2) {
		log_uart(LOGERROR, "Parse args failed.");
		return 0;
	}
	if (speed < minsp || speed > maxsp) {
		log_uart_f(LOGERROR, "Speed not in range: %d, range is %d ~ %d.", speed,
				minsp, maxsp);
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

static HAL_StatusTypeDef can_send_feedback(HAL_StatusTypeDef send_res) {
	if (send_res == HAL_OK) {
		log_uart_f(LOGDEBUG, "CAN Sent.");
	} else {
		log_uart_f(LOGERROR, "CAN Send failed.");
	}
	return send_res;
}

static int cn1_hold_count = 0;
void detect_cn1() {
	// if CN1 signal is low
	if (!(CN1_GPIO_Port->IDR & CN1_Pin)) {
		// avoid shake - hold at least 50ms to trigger
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

void can_cmd_run(uint8_t* data, uint8_t len) {
	// no lm_commit here, program will commit on next cycle.
	if (len >= 2) {
		uint8_t ver = data[0], cmd = data[1];
		if (ver > CAN_CMD_VER) {
			log_uart_f(LOGWARN, "CAN Message Version Higher than me.");
		}
		if (cmd == CAN_CMD_LM) {
			// 按照 cmd_run 里面的编码方法重新组装消息
			lmcmd.type = data[2];
			lmcmd.pos_speed = (int32_t) ((data[3] << 24) + (data[4] << 16)
					+ (data[5] << 8) + (data[6]));
			lmcmd.dir_hard = data[7];
		} else if (cmd == CAN_CMD_STRING) {
			char buf[8] = { 0 };
			memcpy(buf, data + 2, len - 2);
			log_uart_f(LOGINFO, "CAN Received: %s", buf);
		}
	} else {
		log_uart_f(LOGWARN, "CAN Invalid Message: Too Short.");
	}
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
