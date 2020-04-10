#ifndef __FLASH_H__
#define __FLASH_H__
#include "stm32f1xx.h"

uint8_t flash_load_machineid();
HAL_StatusTypeDef flash_save_machineid(uint8_t id);

#endif