#ifndef __FLASH_H__
#define __FLASH_H__
#include "stm32f1xx.h"

void flash_init();
void flash_deinit();
_Bool flash_load_machineid(uint16_t *id);
HAL_StatusTypeDef flash_save_machineid(uint16_t id);
_Bool flash_load_wifissid(char* ssid);
HAL_StatusTypeDef flash_save_wifissid(const char* ssid);
_Bool flash_load_wifipwd(char* pwd);
HAL_StatusTypeDef flash_save_wifipwd(const char* pwd);
_Bool flash_load_tcpip(char* ip);
HAL_StatusTypeDef flash_save_tcpip(const char* ip);
_Bool flash_load_tcpport(uint16_t *port);
HAL_StatusTypeDef flash_save_tcpport(uint16_t port);

_Bool flash_load_motorlimitin(int32_t* in);
_Bool flash_load_motorlimitout(int32_t* out);
HAL_StatusTypeDef flash_save_motor_limit_in(int32_t in);
HAL_StatusTypeDef flash_save_motor_limit_out(int32_t out);

#endif