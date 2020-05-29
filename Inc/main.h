/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void cmd_serial_int(UART_HandleTypeDef *huart);
void wifi_serial_int(UART_HandleTypeDef *huart);
int cn1_pressed();
int sw2_pressed();
int sw3_pressed();
struct inputbuf *getuserbuf();
void load_configurations();
void stm_chip_reset(uint32_t tick);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define L6470Flag_Pin GPIO_PIN_1
#define L6470Flag_GPIO_Port GPIOC
#define L6470Busy_Pin GPIO_PIN_2
#define L6470Busy_GPIO_Port GPIOC
#define Break1_Pin GPIO_PIN_3
#define Break1_GPIO_Port GPIOC
#define SW2_Pin GPIO_PIN_0
#define SW2_GPIO_Port GPIOA
#define SW3_Pin GPIO_PIN_1
#define SW3_GPIO_Port GPIOA
#define WifiTX_Pin GPIO_PIN_2
#define WifiTX_GPIO_Port GPIOA
#define WifiRx_Pin GPIO_PIN_3
#define WifiRx_GPIO_Port GPIOA
#define L6470CS_Pin GPIO_PIN_4
#define L6470CS_GPIO_Port GPIOA
#define Break2_Pin GPIO_PIN_0
#define Break2_GPIO_Port GPIOB
#define CN1_Pin GPIO_PIN_6
#define CN1_GPIO_Port GPIOC
#define Led2_Pin GPIO_PIN_9
#define Led2_GPIO_Port GPIOC
#define Led1_Pin GPIO_PIN_8
#define Led1_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
