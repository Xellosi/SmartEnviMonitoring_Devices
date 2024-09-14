/*
 * utils.c
 *
 *  Created on: Jul 24, 2024
 *      Author: hung
 */

#include "utils.h"
#include "stm32f4xx.h"

static char HexChrs[] = "0123456789ABCDEF";

static void _intToHex(uint32_t num, char *txt);

void Read_Device_Uid(char *id) {
	uint32_t id0 = HAL_GetUIDw0();
	uint32_t id1 = HAL_GetUIDw1();
	uint32_t id2 = HAL_GetUIDw2();
	_intToHex(id0, id);
	_intToHex(id1, id + 8);
	_intToHex(id2, id + 16);
}

void Uint32ToStr(uint32_t num, char **strptr, int *len) {
	uint32_t n = num;
	if (num == 0) {
		(*strptr)[0] = '0';
		*len = 1;
	}

	int l = 0;
	while (n > 0) {
		(*strptr)[l++] = (n % 10) + '0';
		n = n / 10;
	}
	*len = l;
}

static void _intToHex(uint32_t num, char *txt) {
	for (int i = 0; i < 8; i++) {
		char c = HexChrs[num & (15U)];
		num = num >> 4;
		*(txt + 7 - i) = c;
	}
}
