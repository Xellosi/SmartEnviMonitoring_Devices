/*
 * esp32at_driver.h
 *
 *  Created on: Jul 6, 2024
 *      Author: hung
 */

#ifndef INC_ESP32AT_DRIVER_H_
#define INC_ESP32AT_DRIVER_H_

#include <stdbool.h>
#include "stm32f4xx.h"

typedef enum {
	ESP_RESULT_SUCCESS,
	ESP_RESULT_ERROR,
	ESP_RESULT_BUSY,
} ESP32_CMD_RESULT;

typedef enum {
	ESP_HTTP,
	ESP_MQTT,
} ESP_PROTOCOL;

typedef enum {
	ESP_HTTP_HEAD = 1,
	ESP_HTTP_GET = 2,
	ESP_HTTP_POST = 3,
	ESP_HTTP_PUT = 4,
	ESP_HTTP_DELETE = 5,
} ESP_HTTP_REQ_TYPE;

typedef enum {
	ESP_HTTP_APP_FORM = 0,
	ESP_HTTP_APP_JSON = 1,
	ESP_HTTP_MUTI_FORM = 2,
	ESP_HTTP_XML = 3,
} ESP_HTTP_CONTENT_TYPE;

typedef enum {
	ESP_HTTP_TLS = 1,
	ESP_HTTP_SSL = 2,
} ESP_HTTP_TRAN_TYPE;

#define ESP_SEND_TIMEOUT 2000
#define ESP_MQTT_QOS 2;
#define ESP_CMD_LEN 256

extern const char ESP_AT[];
extern const char ESP_ATE0[];

extern const char ESP_RESP_OK[];
extern const char ESP_RESP_FAIL[];
extern const char ESP_RESP_BUSY[];

extern const char ESP_WIFF_CONN[];
extern const char ESP_WIFF_GOTIP[];

extern const char ESP_HTTP_GET_Prefix[];
extern const char ESP_HTTP_CLIENT_Prefix[];

extern const char ESP_HTTP_GET_RES_Prefix[];
extern const char ESP_HTTP_CLIENT_RES_Prefix[];

extern const char ESP_MQTT_CFG_Prefix[];
extern const char ESP_MQTT_CONN_Prefix[];
extern const char ESP_MQTT_Sub_Prefix[];
extern const char ESP_MQTT_Unsub_Prefix[];
extern const char ESP_MQTT_Pub_Prefix[];

extern const char ESP_MQTT_MES_Prefix[];
extern const char ESP_MQTT_REC_CONN_Prefix[];
extern const char ESP_MQTT_REC_DISCONN_Prefix[];

extern const char ESP_HTTPCLIENT_RES_PREFIX[];
extern const char ESP_HTTPGET_RES_PREFIX[];

HAL_StatusTypeDef ESP_Cmd_AT(UART_HandleTypeDef* uart);

HAL_StatusTypeDef ESP_Close_Echo(UART_HandleTypeDef* uart);

HAL_StatusTypeDef ESP_Http_Get(UART_HandleTypeDef* uart, char* url);

HAL_StatusTypeDef ESP_Http_Post(UART_HandleTypeDef* uart, char* url);

HAL_StatusTypeDef ESP_MQTT_Cfg(UART_HandleTypeDef* uart, uint8_t scheme, char* clientId,
		char* username, char* password);

HAL_StatusTypeDef ESP_MQTT_CONN(UART_HandleTypeDef* uart, char* url, char* port);

HAL_StatusTypeDef ESP_MQTT_CLEAN(UART_HandleTypeDef* uart);

HAL_StatusTypeDef ESP_MQTT_Sub(UART_HandleTypeDef* uart, char* topic);

HAL_StatusTypeDef ESP_MQTT_Unsub(UART_HandleTypeDef* uart, char* topic);

HAL_StatusTypeDef ESP_MQTT_Pub(UART_HandleTypeDef* uart, char* topic, char* msg);

HAL_StatusTypeDef Uart_Print_Send(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size, uint32_t Timeout);

#endif /* INC_ESP32AT_DRIVER_H_ */
