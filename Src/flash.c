#include "flash.h"
#include "stm32f1xx.h"
#include "log_uart.h"
#include <string.h>
#include "config.h"

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
// 261
#define MOTOR_OUT_LIMIT_STORE (FLASH_BASE+(0x41400))
// 262
#define MOTOR_IN_LIMIT_STORE (FLASH_BASE+(0x41800))

void flash_init() {
    if(HAL_FLASH_Unlock() != HAL_OK) {
        logu_s(LOGU_ERROR, "Flash unlock failed.");
    }
}
void flash_deinit() {
    HAL_FLASH_Lock();
}

static _Bool flash_load_string(uint32_t addr, char* dest, uint16_t limit);
static HAL_StatusTypeDef flash_save_data(uint32_t addr, const char *data, uint16_t size);
static HAL_StatusTypeDef flash_erase_save_data(uint32_t addr, const char* data, uint16_t size);

_Bool flash_load_machineid(uint16_t *id) {
    uint16_t tmpid = *(volatile uint16_t*)(MACHINE_ID_STORE);
    if(tmpid == 0 || tmpid == 0xFFFF) {
        return 0;
    } else {
    	*id = tmpid;
        return 1;
    }
}
HAL_StatusTypeDef flash_save_machineid(uint16_t id) {
    return flash_erase_save_data(MACHINE_ID_STORE, (char*)&id, 2);
}
_Bool flash_load_wifissid(char* ssid) {
    return flash_load_string(WIFI_SSID_STORE, ssid, WIFI_STRSIZE);
}
HAL_StatusTypeDef flash_save_wifissid(const char* ssid) {
    return flash_erase_save_data(WIFI_SSID_STORE, ssid, strlen(ssid) + 1);
}
_Bool flash_load_wifipwd(char* pwd) {
    return flash_load_string(WIFI_PWD_STORE, pwd, WIFI_STRSIZE);
}
HAL_StatusTypeDef flash_save_wifipwd(const char* pwd) {
    return flash_erase_save_data(WIFI_PWD_STORE, pwd, strlen(pwd) + 1);
}
_Bool flash_load_tcpip(char* ip) {
    return flash_load_string(WIFI_TCPIP_STORE, ip, WIFI_STRSIZE);
}
HAL_StatusTypeDef flash_save_tcpip(const char* ip) {
    return flash_erase_save_data(WIFI_TCPIP_STORE, ip, strlen(ip) + 1);
}
_Bool flash_load_tcpport(uint16_t *port) {
    uint16_t tmpport = *(volatile uint16_t*)(WIFI_TCPPORT_STORE);
    if(tmpport == 0xFFFF) {
        return 0;
    } else {
        *port = tmpport;
        return 1;
    }
}
HAL_StatusTypeDef flash_save_tcpport(uint16_t port) {
    return flash_erase_save_data(WIFI_TCPPORT_STORE, (char*)&port, 2);
}

_Bool flash_load_motorlimitin(int32_t* in) {
    int32_t tmpin;
    tmpin = *(volatile int32_t*)(MOTOR_IN_LIMIT_STORE);
    if((uint32_t)tmpin == 0xFFFFFFFF) {
        return 0;
    } else {
        *in = tmpin;
        return 1;
    }
}
_Bool flash_load_motorlimitout(int32_t* out) {
    int32_t tmpout;
    tmpout = *(volatile int32_t*)(MOTOR_OUT_LIMIT_STORE);
    if((uint32_t)tmpout == 0xFFFFFFFF) {
        return 0;
    }
    *out = tmpout;
    return 1;
}
HAL_StatusTypeDef flash_save_motor_limit_in(int32_t in) {
    return flash_erase_save_data(MOTOR_IN_LIMIT_STORE, (char*)&in, 4);
}
HAL_StatusTypeDef flash_save_motor_limit_out(int32_t out) {
    return flash_erase_save_data(MOTOR_OUT_LIMIT_STORE, (char*)&out, 4);
}

// 这个函数非常依赖写入的时候最后一个 NULL 字符必须成功，不然因为Flash复原的时候都是FF，strncpy停不下来。
static _Bool flash_load_string(uint32_t addr, char* dest, uint16_t limit) {
    // 如果这个地方是空的，那么直接返回
    if(*(volatile uint8_t*)addr == 0xFF) {
        return 0;
    }
    strncpy(dest, (const char*)addr, limit);
    return 1;
}

// 一定要保证这里的端序和处理器一样。
static HAL_StatusTypeDef flash_save_data(uint32_t addr, const char *data, uint16_t size) {
    const uint16_t sizecut = size & (~0x1);
    const char* const fin = data + sizecut;
    while(data < fin) {
        uint16_t unit = data[0];
        unit |= data[1] << 8;
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

// 如果不能直接写入，那么擦除addr开始的一整页然后重新尝试写入。
static HAL_StatusTypeDef flash_erase_save_data(uint32_t addr, const char* data, uint16_t size) {
    if(flash_save_data(addr, data, size) != HAL_OK) {
        FLASH_EraseInitTypeDef erase;
        uint32_t eraseerror;
        erase.TypeErase = FLASH_TYPEERASE_PAGES;
        erase.Banks = FLASH_BANK_1;
        erase.PageAddress = addr;
        erase.NbPages = 1;
        HAL_StatusTypeDef st = HAL_FLASHEx_Erase(&erase, &eraseerror);
        if(st != HAL_OK) {
            return st;
        }
        return flash_save_data(addr, data, size);
    } else {
        return HAL_OK;
    }
}
