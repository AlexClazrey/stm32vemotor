#include "util.h"
#include <string.h>

// Utility Functions

// Subtraction with custom overflow value,
// from is included, to is not included, return virtually (to - from).
// value range is [0, overval - 1], can tolerate unsigned negative input [-overval, -1].
// input larger or equals to overval is considered as negative.
uint32_t diffu(uint32_t from, uint32_t to, uint32_t overval) {
	if (to > overval)
		to += overval;
	if (from > overval)
		from += overval;
	if (to >= from)
		return to - from;
	else
		return to + overval - from ;
}

// Addition with custom overflow value,
// value range is [0, overval - 1], can tolerate unsigned negative input [-overval, -1].
// input larger or equals to overval is considered as negative.
uint32_t addu(uint32_t a1, uint32_t a2, uint32_t overval) {
	if (a1 > overval)
		a1 += overval;
	if (a2 > overval)
		a2 += overval;
	uint32_t res = a1 + a2;
	return res >= overval ? res - overval : res;
}

// Empty Ascending FIFO/Circular Array to Simple Array
// return -1 if not destination size is not big enough, nothing will be changed.
// otherwise return copied byte counts.
// cannot tolerate negative input
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
// cannot tolerate negative input
//
int arrtocycarr(char* dest, size_t start, size_t end, size_t destsize, const char* src, size_t len) {
	if(end >= destsize || start >= destsize || len > destsize) {
		return -1;
	}
	if(end < start) {
		size_t s1 = destsize - start;
		size_t st = s1 + end;
		if(st < len) {
			return -1;
		}
		memcpy(dest+start, src, s1);
		memcpy(dest, src + s1, len - s1);
		return len;
	} else {
		size_t st = end - start;
		if(st < len) {
			return -1;
		}
		memcpy(dest + start, src, len);
		return len;
	}
}

// iteration in circular array
// start is included, end is not included.
// if argument func returns non-zero value, iteration will be aborted and that value will be returned
// cannot tolerate negative input
int cycarriter(void* arr, const size_t unit, const size_t start, const size_t end, const size_t arrsize, int (*func)(void*, size_t)) {
	if(end >= arrsize || start >= arrsize) {
		return -1;
	}
	void* cursor = arr + start * unit;
	if(end < start) {
		for(size_t i = start; i < arrsize; i++) {
			int r = (*func)(cursor, i);
			if(r)
				return r;
			cursor += unit;
		}
		cursor = arr;
		for(size_t i = 0; i < end; i++) {
			int r = (*func)(cursor, i);
			if(r)
				return r;
			cursor += unit;
		}
	} else {
		for(size_t i = start; i < end; i++) {
			int r = (*func)(cursor, i);
			if(r)
				return r;
			cursor += unit;
		}}
	return 0;
}

// test if an index is in between two indexes in a circular array
// cannot tolerate negative input
int cycbetween(uint32_t start, uint32_t target, uint32_t end, uint32_t size) {
	if(start >= size || end >= size || target >= size)
		return 0;
	if(end < start) {
		return target <= end || start <= target;
	} else {
		return start <= target && target <= end;
	}
}
