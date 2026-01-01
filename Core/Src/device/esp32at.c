/*
 * esp32at.c
 *
 *  Created on: Jul 6, 2024
 *      Author: hung
 */
#include <string.h>
#include "esp32at.h"
#include "lwesp/lwesp.h"
#include "lwesp/lwesp_input.h"
#include "lwesp/lwesp_mem.h"

#ifndef LWESP_MEM_SIZE
#define LWESP_MEM_SIZE 0x4000
#endif

static UART_HandleTypeDef* esp_uart = NULL;
static uint8_t lwesp_memory[LWESP_MEM_SIZE];
static uint8_t lwesp_memory_ready = 0;

void ESP32AT_SetUart(UART_HandleTypeDef* uart) {
	esp_uart = uart;
}

void ESP32AT_Input(const uint8_t* data, size_t len) {
	if (data == NULL || len == 0) {
		return;
	}
	lwesp_input_process(data, len);
}

static size_t ESP32AT_Send(const void* data, size_t len) {
	if (esp_uart == NULL || data == NULL || len == 0) {
		return 0;
	}
	if (HAL_UART_Transmit(esp_uart, (const uint8_t*)data, len, HAL_MAX_DELAY) != HAL_OK) {
		return 0;
	}
	return len;
}

lwespr_t lwesp_ll_init(lwesp_ll_t* ll) {
	if (ll == NULL) {
		return lwespERRPAR;
	}

	if (!lwesp_memory_ready) {
		const lwesp_mem_region_t mem_regions[] = {
				{lwesp_memory, sizeof(lwesp_memory)},
		};
		lwesp_mem_assignmemory(mem_regions, sizeof(mem_regions) / sizeof(mem_regions[0]));
		lwesp_memory_ready = 1;
	}

	ll->send_fn = ESP32AT_Send;
	ll->reset_fn = NULL;
	return lwespOK;
}

lwespr_t lwesp_ll_deinit(lwesp_ll_t* ll) {
	(void)ll;
	return lwespOK;
}
