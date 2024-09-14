/*
 * esp32at_driver.c
 *
 *  Created on: Jul 6, 2024
 *      Author: hung
 */
#include <stdio.h>
#include <string.h>
#include "esp32at.h"

const char ESP_AT[] = "AT\r\n";
const char ESP_ATE0[] = "ATE0\r\n";

const char ESP_RESP_OK[] = "OK";
const char ESP_RESP_FAIL[] = "Fail";
const char ESP_RESP_BUSY[] = "busy";

const char ESP_WIFF_CONN[] = "WIFI CONNECTED";
const char ESP_WIFF_GOTIP[] = "WIFI GOT IP";

const char ESP_HTTP_GET_Prefix[] = "AT+HTTPCGET=";



const char ESP_HTTP_CLIENT_Prefix[] = "AT+HTTPCLIENT=";

const char ESP_HTTP_GET_RES_Prefix[] = "+HTTPGETSIZE";
const char ESP_HTTP_CLIENT_RES_Prefix[] = "+HTTPCLIENT";

const char ESP_MQTT_CFG_Prefix[] = "AT+MQTTUSERCFG=0";
const char ESP_MQTT_CONN_Prefix[] = "AT+MQTTCONN=0";
const char ESP_MQTT_CLEAN_Prefix[] = "AT+MQTTCLEAN=0";
const char ESP_MQTT_Sub_Prefix[] = "AT+MQTTSUB=0";
const char ESP_MQTT_Unsub_Prefix[] = "AT+MQTTUNSUB=0";
const char ESP_MQTT_Pub_Prefix[] = "AT+MQTTPUB=0";

const char ESP_MQTT_MES_Prefix[] = "+MQTTSUBRECV:0";
const char ESP_MQTT_REC_CONN_Prefix[] = "+MQTTCONNECTED";
const char ESP_MQTT_REC_DISCONN_Prefix[] = "+MQTTDISCONNECTED";

const char ESP_HTTPCLIENT_RES_PREFIX[] = "+HTTPCLIENT:";
const char ESP_HTTPGET_RES_PREFIX[] = "+HTTPCGET:";

HAL_StatusTypeDef Uart_Print_Send(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t size, uint32_t timeout){
	printf("send %d: %s\n",size ,pData);
	return HAL_UART_Transmit(huart, pData, size, timeout);
}


HAL_StatusTypeDef ESP_Cmd_AT(UART_HandleTypeDef *uart) {
	return Uart_Print_Send(uart, (uint8_t*) ESP_AT, strlen(ESP_AT),
			ESP_SEND_TIMEOUT);
}

HAL_StatusTypeDef ESP_Close_Echo(UART_HandleTypeDef *uart) {
	return Uart_Print_Send(uart, (uint8_t*) ESP_ATE0, strlen(ESP_ATE0),
			ESP_SEND_TIMEOUT);
}

HAL_StatusTypeDef ESP_Http_Get(UART_HandleTypeDef *uart, char *url) {
	char temp[ESP_CMD_LEN];
	uint16_t len = strlen(ESP_HTTP_GET_Prefix);
	memcpy(temp, ESP_HTTP_GET_Prefix, len);

	temp[len++] = '"';

	int urllen = strlen(url);
	memcpy(temp + len, url, urllen);
	len += urllen;

	temp[len++] = '"';

	temp[len++] = '\r';
	temp[len++] = '\n';

	temp[len] = '\0';

	return Uart_Print_Send(uart, (uint8_t*) temp, len, ESP_SEND_TIMEOUT);
}

HAL_StatusTypeDef ESP_Http_Post(UART_HandleTypeDef *uart, char *url) {
	char temp[ESP_CMD_LEN];
	uint16_t len = strlen(ESP_HTTP_CLIENT_Prefix);
	memcpy(temp, ESP_HTTP_CLIENT_Prefix, len);

	temp[len++] = '3';
	temp[len++] = ',';
	temp[len++] = '0';
	temp[len++] = ',';

	temp[len++] = '"';
	int urllen = strlen(url);
	memcpy(temp + len, url, urllen);
	len += urllen;
	temp[len++] = '"';

	temp[len++] = ',';
	temp[len++] = ',';
	temp[len++] = ',';
	temp[len++] = '1';

	temp[len++] = ',';

	temp[len++] = '"';
	temp[len++] = '"';

	temp[len++] = '\r';
	temp[len++] = '\n';
	temp[len] = '\0';

	return Uart_Print_Send(uart, (uint8_t*) temp, len, ESP_SEND_TIMEOUT);
}

HAL_StatusTypeDef ESP_MQTT_Cfg(UART_HandleTypeDef *uart, uint8_t scheme,
		char *clientId, char *username, char *password) {
	char temp[ESP_CMD_LEN];
	uint16_t len = strlen(ESP_MQTT_CFG_Prefix);
	memcpy(temp, ESP_MQTT_CFG_Prefix, len);

	temp[len++] = ',';

	temp[len++] = scheme + '0';

	temp[len++] = ',';

	temp[len++] = '"';
	int clientLen = strlen(clientId);
	memcpy(temp + len, clientId, clientLen);
	len += clientLen;
	temp[len++] = '"';

	temp[len++] = ',';

	temp[len++] = '"';
	int usernameLen = strlen(username);
	memcpy(temp + len, username, usernameLen);
	len += usernameLen;
	temp[len++] = '"';

	temp[len++] = ',';

	temp[len++] = '"';
	int passwordLen = strlen(password);
	memcpy(temp + len, password, passwordLen);
	len += passwordLen;
	temp[len++] = '"';

	temp[len++] = ',';

	temp[len++] = '0';

	temp[len++] = ',';

	temp[len++] = '0';

	temp[len++] = ',';

	temp[len++] = '"';
	temp[len++] = '"';

	temp[len++] = '\r';
	temp[len++] = '\n';

	temp[len] = '\0';
	return Uart_Print_Send(uart, (uint8_t*) temp, len, ESP_SEND_TIMEOUT);
}

HAL_StatusTypeDef ESP_MQTT_CONN(UART_HandleTypeDef *uart, char *url, char *port) {
	char temp[ESP_CMD_LEN];
	uint16_t len = strlen(ESP_MQTT_CONN_Prefix);
	memcpy(temp, ESP_MQTT_CONN_Prefix, len);

	temp[len++] = ',';

	temp[len++] = '"';
	int urllen = strlen(url);
	memcpy(temp + len, url, urllen);
	len += urllen;
	temp[len++] = '"';

	temp[len++] = ',';

	int portlen = strlen(port);
	memcpy(temp + len, port, portlen);
	len += portlen;

	temp[len++] = ',';

	temp[len++] = '1';

	temp[len++] = '\r';
	temp[len++] = '\n';

	temp[len] = '\0';
	return Uart_Print_Send(uart, (uint8_t*) temp, len, ESP_SEND_TIMEOUT);
}

HAL_StatusTypeDef ESP_MQTT_CLEAN(UART_HandleTypeDef* uart){
	char temp[ESP_CMD_LEN];
	uint16_t len = strlen(ESP_MQTT_CLEAN_Prefix);
	memcpy(temp, ESP_MQTT_CLEAN_Prefix, len);

	temp[len++] = '\r';
	temp[len++] = '\n';

	temp[len] = '\0';
	return Uart_Print_Send(uart, (uint8_t*) temp, len, ESP_SEND_TIMEOUT);
}

HAL_StatusTypeDef ESP_MQTT_Sub(UART_HandleTypeDef *uart, char *topic) {
	char temp[ESP_CMD_LEN];
	uint16_t len = strlen(ESP_MQTT_Sub_Prefix);
	memcpy(temp, ESP_MQTT_Sub_Prefix, len);
	temp[len++] = ',';

	temp[len++] = '"';
	int topiclen = strlen(topic);
	memcpy(temp + len, topic, topiclen);
	len += topiclen;
	temp[len++] = '"';

	temp[len++] = ',';
	//qos = 1, at least one
	temp[len++] = '1';

	temp[len++] = '\r';
	temp[len++] = '\n';

	temp[len] = '\0';
	return Uart_Print_Send(uart, (uint8_t*) temp, len, ESP_SEND_TIMEOUT);
}

HAL_StatusTypeDef ESP_MQTT_Unsub(UART_HandleTypeDef *uart, char *topic) {
	char temp[ESP_CMD_LEN];
	uint16_t len = strlen(ESP_MQTT_Unsub_Prefix);
	memcpy(temp, ESP_MQTT_Unsub_Prefix, len);
	temp[len++] = ',';

	temp[len++] = '"';
	int topiclen = strlen(topic);
	memcpy(temp + len, topic, topiclen);
	len += topiclen;
	temp[len++] = '"';

	temp[len++] = '\r';
	temp[len++] = '\n';

	temp[len] = '\0';
	return Uart_Print_Send(uart, (uint8_t*) temp, len, ESP_SEND_TIMEOUT);
}

HAL_StatusTypeDef ESP_MQTT_Pub(UART_HandleTypeDef *uart, char *topic, char *msg) {
	char temp[ESP_CMD_LEN];
	uint16_t len = strlen(ESP_MQTT_Pub_Prefix);
	memcpy(temp, ESP_MQTT_Pub_Prefix, len);
	temp[len++] = ',';

	temp[len++] = '"';
	int topiclen = strlen(topic);
	memcpy(temp + len, topic, topiclen);
	len += topiclen;
	temp[len++] = '"';

	temp[len++] = ',';

	temp[len++] = '"';
	int msglen = strlen(msg);
	memcpy(temp + len, msg, msglen);
	len += msglen;
	temp[len++] = '"';

	temp[len++] = ',';

	temp[len++] = '1';

	temp[len++] = ',';

	temp[len++] = '"';
	temp[len++] = '"';

	temp[len++] = '\r';
	temp[len++] = '\n';

	temp[len] = '\0';
	return Uart_Print_Send(uart, (uint8_t*) temp, len, ESP_SEND_TIMEOUT);
}

void _AppendCmdEnd(char **cmd) {
	(*cmd)[0] = '\r';
	(*cmd)[1] = '\n';
}
