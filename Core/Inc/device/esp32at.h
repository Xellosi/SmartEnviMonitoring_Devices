/*
 * esp32at.h
 *
 *  Created on: Jul 6, 2024
 *      Author: hung
 */

#ifndef INC_ESP32AT_DRIVER_H_
#define INC_ESP32AT_DRIVER_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "stm32f4xx.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	ESP32AT_HTTP_METHOD_HEAD = 1,
	ESP32AT_HTTP_METHOD_GET = 2,
	ESP32AT_HTTP_METHOD_POST = 3,
	ESP32AT_HTTP_METHOD_PUT = 4,
	ESP32AT_HTTP_METHOD_DELETE = 5
} Esp32AtHttpMethod;

void ESP32AT_SetUart(UART_HandleTypeDef* uart);
void ESP32AT_Input(const uint8_t* data, size_t len);
size_t ESP32AT_SendRaw(const uint8_t* data, size_t len);
size_t ESP32AT_SendCommand(const char* cmd);
bool ESP32AT_SendCommandWait(const char* cmd, uint32_t timeout_ms);
bool ESP32AT_Init(const char* wifi_ssid, const char* wifi_pass, uint32_t timeout_ms);
size_t ESP32AT_HttpClientRequest(Esp32AtHttpMethod method, const char* url, const char* body, size_t body_len,
		char* response, size_t response_len, int* status_code, uint32_t timeout_ms);
bool ESP32AT_TcpConnect(const char* host, uint16_t port, uint32_t timeout_ms);
bool ESP32AT_TcpSend(const uint8_t* data, size_t len, uint32_t timeout_ms);
size_t ESP32AT_TcpReceive(char* out, size_t out_len, uint32_t timeout_ms);
bool ESP32AT_TcpClose(uint32_t timeout_ms);
bool ESP32AT_MqttConfig(const char* client_id, const char* user, const char* pass, uint16_t keep_alive);
bool ESP32AT_MqttConnect(const char* host, uint16_t port, uint32_t timeout_ms);
bool ESP32AT_MqttSubscribe(const char* topic, uint8_t qos, uint32_t timeout_ms);
bool ESP32AT_MqttPublish(const char* topic, const char* payload, uint8_t qos, bool retain, uint32_t timeout_ms);
bool ESP32AT_MqttDisconnect(uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* INC_ESP32AT_DRIVER_H_ */
