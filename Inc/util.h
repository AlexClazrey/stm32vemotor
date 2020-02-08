#ifndef __UTIL_H__
#define __UTIL_H__

#include "stm32f1xx.h"

#define true 1
#define false 0

uint32_t diffu(uint32_t from, uint32_t to, uint32_t overval);
uint32_t addu(uint32_t a1, uint32_t a2, uint32_t overval);
int cycarrtoarr(char* dest, size_t destsize, const char* src, size_t start, size_t end, size_t srcsize);
int arrtocycarr(char* dest, size_t start, size_t end, size_t destsize, const char* src, size_t len);

#endif
