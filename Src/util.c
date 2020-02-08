#include "util.h"
#include <string.h>

// Utility Functions

// Subtraction with custom overflow value, value range is [0, overval - 1].
uint32_t diffu(uint32_t from, uint32_t to, uint32_t overval) {
	if (to > from)
		return to - from;
	else
		return to - from + overval;
}

// Addition with custom overflow value, value range is [0, overval - 1].
uint32_t addu(uint32_t a1, uint32_t a2, uint32_t overval) {
	uint32_t res = a1 + a2;
	return res >= overval ? res % overval : res;
}

// Empty Ascending FIFO/Circular Array to Simple Array
// return -1 if not destination size is not big enough, nothing will be changed.
// otherwise return copied byte counts.
//
int cycarrtoarr(char* dest, size_t destsize, const char* src, size_t start, size_t end, size_t srcsize) {
	if(start >= srcsize || end >= srcsize) {
		return -1;
	}
	if (end < start) {
		size_t s1 = srcsize - start;
		size_t st = s1 + end;
		if (st > destsize) {
			return -1;
		} else {
			memcpy(dest, src + start, s1);
			memcpy(dest + s1, src, end);
			return st;
		}
	} else {
		size_t st = end - start;
		if (st > destsize) {
			return -1;
		} else {
			memcpy(dest, src + start, st);
			return st;
		}
	}
}

// Simple Array to Empty Ascending FIFO/Circular Array
// from dest[start](included) to dest[end](excluded) marks the available empty space.
// Notice if you want to copy till a Circular Array is full, parameter end should be (start-1) and corresponding copy length;
// Two arrays must not overlap.
//
int arrtocycarr(char* dest, size_t start, size_t end, size_t destsize, const char* src, size_t len) {
	if(end >= destsize || start >= destsize || len > destsize) {
		return -1;
	}
	if(start < end) {
		size_t st = end - start;
		if(st < len) {
			return -1;
		}
		memcpy(dest + start, src, len);
		return len;
	} else {
		size_t s1 = destsize - start;
		size_t st = s1 + end;
		if(st < len) {
			return -1;
		}
		memcpy(dest+start, src, s1);
		memcpy(dest, src + s1, len - s1);
		return len;
	}
}
