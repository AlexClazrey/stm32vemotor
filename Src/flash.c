
#include "stm32f1xx.h"
#include "log_uart.h"
#include <string.h>

// offset 256KB
#define MACHINE_ID_STORE (FLASH_BASE+(0x40000))
// offset 257KB
#define WIFI_SSID_STORE (FLASH_BASE+(0x40400))
// 258
#define WIFI_PWD_STORE (FLASH_BASE+(0x40800))
// 259
#define WIFI_TCPIP_STORE (FLASH_BASE+(0x40c00))
// 260
#define WIFI_TCPPORT_STORE (FLASH_BASE+(0x41000))

void flash_init() {
    if(HAL_FLASH_Unlock() != HAL_OK) {
        logu_s(LOGU_ERROR, "Flash unlock failed.");
    }
}
void flash_deinit() {
    HAL_FLASH_Lock();
}

uint8_t flash_load_machineid() {
    return *(volatile uint8_t*)(MACHINE_ID_STORE);
}

HAL_StatusTypeDef flash_save_machineid(uint8_t id) {
    logu_s(LOGU_TRACE, "Flash id start.");
    HAL_StatusTypeDef st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, MACHINE_ID_STORE, id);
    logu_s(LOGU_TRACE, "Flash id complete.");
    return st;
}

static void flash_load_string(uint32_t addr, uint8_t* dest, uint16_t limit);
static HAL_StatusTypeDef flash_save_data(uint32_t addr, uint8_t *data, uint16_t size);

void flash_load_ssid(char* ssid) {
    flash_load_string(WIFI_SSID_STORE, ssid, 128);
}
HAL_StatusTypeDef flash_save_ssid(char* ssid) {
    return flash_save_data(WIFI_SSID_STORE, ssid, strlen(ssid));
}
void flash_load_pwd(char* pwd) {
    flash_load_string(WIFI_PWD_STORE, pwd, 128);
}
HAL_StatusTypeDef flash_save_pwd(char* pwd) {
    return flash_save_data(WIFI_PWD_STORE, pwd, strlen(pwd));
}
void flash_load_tcpip(char* ip) {
    flash_load_string(WIFI_TCPIP_STORE, ip, 128);
}
void flash_save_tcpip(char* ip) {
    return flash_save_data(WIFI_TCPIP_STORE, ip, strlen(ip));
}
uint16_t flash_load_tcpport() {
    return *(volatile uint16_t*)(WIFI_TCPPORT_STORE);
}
void flash_save_tcpport(uint16_t port) {
    return HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, WIFI_TCPPORT_STORE, port);
}


static void flash_load_string(uint32_t addr, uint8_t* dest, uint16_t limit) {
    strncpy(dest, (const char*)addr, limit);
}

static HAL_StatusTypeDef flash_save_data(uint32_t addr, uint8_t *data, uint16_t size) {
    uint16_t sizecut = size | 0x0;
    uint8_t *fin = data + sizecut;
    while(data < fin) {
        uint16_t unit = data[0];
        unit += data[1] << 8;
        HAL_StatusTypeDef st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr, unit);
        if(st != HAL_OK) {
            return st;
        }
        data += 2;
        addr += 2;
    }
    if (sizecut < size) {
        uint16_t unit = data[0];
        HAL_StatusTypeDef st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr, unit);
        if(st != HAL_OK) {
            return st;
        }
    }
    return HAL_OK;
}
