/*
 * esp32at.h
 *
 *  Created on: Jul 6, 2024
 *      Author: hung
 */

#ifndef INC_ESP32AT_DRIVER_H_
#define INC_ESP32AT_DRIVER_H_

#include <stddef.h>
#include <stdint.h>
#include "stm32f4xx.h"

#ifdef __cplusplus
extern "C" {
#endif

void ESP32AT_SetUart(UART_HandleTypeDef* uart);
void ESP32AT_Input(const uint8_t* data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* INC_ESP32AT_DRIVER_H_ */
