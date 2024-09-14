/*
 * http_comm.h
 *
 *  Created on: Jul 6, 2024
 *      Author: hung
 */

#ifndef INC_NET_COMM_H_
#define INC_NET_COMM_H_

#include <stdbool.h>

#include "esp32at.h"
#include "utils.h"

#define LOGIN_PREFIX ("login")
#define RES_SUCC ("succ")

typedef struct{
	char* HttpDeviceUrl;
	char* HttpWeatherUrl;
	char* ServerIp;
	char* MQTTPort;
	char DeviceId[DEVICE_UID_LEN];
	TIM_HandleTypeDef* HtimMs;
}CommHandle_t;

typedef struct {
	uint8_t Year;
	uint8_t Month;
	uint8_t Day;

	uint8_t Hours;
	uint8_t Minutes;
	uint8_t Seconds;
}DateTime_t;

static const char MsgStart = '#';
static const char FieldSeperater = '_';
static const char KeyValueSeperater = '@';
static const char MsgEnd = ';';

extern const char SUFFIX_REQ[];
extern const char SUFFIX_RES[];

extern const char POST_SUCCESS[];

ErrorStatus Build_CommHandle(CommHandle_t* h, char* httpDeviceUrl, char* httpWeatherUrl,
		char* serverIp, char* mqttPort, char deviceID[DEVICE_UID_LEN], TIM_HandleTypeDef* htim);

ErrorStatus Try_Parse_Time(char* str, DateTime_t* data);

ErrorStatus Post_Login(UART_HandleTypeDef* uart, CommHandle_t* hcomm);

ErrorStatus Post_Logout(UART_HandleTypeDef* uart, CommHandle_t* hcomm);

ErrorStatus Get_CurrentTime(UART_HandleTypeDef* uart, CommHandle_t* hcomm, DateTime_t* dt);

ErrorStatus MQTT_Login(UART_HandleTypeDef* uart, CommHandle_t* hcomm);

ErrorStatus Build_WeatherReportQuery(char* str, CommHandle_t* hcomm, uint8_t tempC, uint8_t humidty, char* deviceID);
#endif /* INC_NET_COMM_H_ */
