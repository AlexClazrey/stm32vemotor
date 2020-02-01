#include "stm32f1xx.h"

uint32_t diffu(uint32_t from, uint32_t to, uint32_t overval);
uint32_t addu(uint32_t a1, uint32_t a2, uint32_t overval);
int cycarrcpy(char* dest, char* buf, int start, int end, int bufsize, int cpylimit);
