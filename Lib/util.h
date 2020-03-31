#ifndef __UTIL_H__
#define __UTIL_H__

#include "stm32f1xx.h"

#define true 1
#define false 0
#define ABS(x) ((x)>0?(x):(-(x)))

// Substraction with custom overflow value,
// input value range is [0, overval - 1].
static inline uint32_t diffup(uint32_t from, uint32_t to, uint32_t overval) {
    if (to >= from)
		return to - from;
	else
		return to + overval - from ;
}
uint32_t diffu(uint32_t from, uint32_t to, uint32_t overval);
uint32_t addu(uint32_t a1, uint32_t a2, uint32_t overval);
int cycarrtoarr(char* dest, size_t destsize, const char* src, size_t start, size_t end, size_t srcsize);
int arrtocycarr(char* dest, size_t start, size_t end, size_t destsize, const char* src, size_t len);
int cycarriter(void* arr, const size_t unit, const size_t start, const size_t end, const size_t arrsize, int (*func)(void*, size_t));
int cycbetween(uint32_t start, uint32_t target, uint32_t end, uint32_t size);

static inline _Bool ule_tolerance(uint32_t big, uint32_t small, uint32_t tolerance) {
	if(small > tolerance)
		return big >= small - tolerance;
	else
		return 1;
}

#endif
