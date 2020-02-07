#include "util.h"
#include <string.h>

// Utility Functions

// 照顾到 Overflow 的差值计算，数据范围是 [0, overval - 1]
uint32_t diffu(uint32_t from, uint32_t to, uint32_t overval) {
	if (to > from)
		return to - from;
	else
		return to - from + overval;
}

uint32_t addu(uint32_t a1, uint32_t a2, uint32_t overval) {
	uint32_t res = a1 + a2;
	return res >= overval ? res - overval : res;
}

// Empty Ascending FIFO/Circular Array
int cycarrcpy(char* dest, char* buf, int start, int end, int bufsize, int cpylimit) {
	if(start >= bufsize || end >= bufsize) {
		return -1;
	}
	if (end < start) {
		int s1 = bufsize - start;
		int st = s1 + end;
		if (st > cpylimit) {
			return -1;
		} else {
			memcpy(dest, buf + start, s1);
			memcpy(dest + s1, buf, end);
			return st;
		}
	} else {
		int st = end - start;
		if (st > cpylimit) {
			return -1;
		} else {
			memcpy(dest, buf + start, end - start);
			return st;
		}
	}
}
