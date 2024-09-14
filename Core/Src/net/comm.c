/*
 * http_comm.c
 *
 *  Created on: Jul 6, 2024
 *      Author: hung
 */

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include "stm32f4xx.h"
#include "esp32at.h"
#include "comm.h"
#include "utils.h"

const char MQTT_CONN[] = "+MQTTCONNECTED";
const char MQTT_DISCONN[] = "+MQTTDISCONNECTED";

const char DATETIME_SP[] = "T";
const char DATE_SP[] = "-";
const char TIME_SP[] = ":";

const char LOGIN[] = "login";
const char LOGOUT[] = "logout";
const char CURRENT_TIME[] = "timecurrent";
const char DEVICEID_KEY[] = "deviceUID";

const char SUFFIX_REQ[] = "_req";
const char SUFFIX_RES[] = "_res";

const char TEMPC[] = "TemperatureC";
const char HUMIDITY[] = "HUMIDITY";

const char POST_SUCCESS[] = "succ";

ErrorStatus Build_CommHandle(CommHandle_t* h, char* httpDeviceUrl, char* httpWeatherUrl, char* serverIp,
		char* mqttPort, char deviceID[DEVICE_UID_LEN], TIM_HandleTypeDef* htimMs)
{
	if (h == NULL){
		return ERROR;
	}

	h -> HttpDeviceUrl = httpDeviceUrl;
	h -> HttpWeatherUrl = httpWeatherUrl;
	h -> ServerIp = serverIp;
	h -> MQTTPort = mqttPort;
	memcpy(h -> DeviceId, deviceID, DEVICE_UID_LEN);
	h -> HtimMs = htimMs;
	return SUCCESS;
}

ErrorStatus Try_Parse_Time(char* str, DateTime_t* data)
{
	//format: yyyy-MM-ddThh:mm:ss
	if (data == NULL){
		return ERROR;
	}

	// parse date
	char* date = strtok(str, DATETIME_SP);
	char* times = strtok(NULL, DATETIME_SP);

	if (date == NULL){
		return ERROR;
	}
	char* yearstr = strtok(date, DATE_SP);
	if (yearstr == NULL){
		return ERROR;
	}
	//20XX => shift 2 postion
	int year = atoi(yearstr + 2);
	if (year == 0){
		return ERROR;
	}
	data -> Year = year;

	char* monthstr = strtok(NULL, DATE_SP);
	if (monthstr == NULL){
		return ERROR;
	}
	int month = atoi(monthstr);
	if (month == 0){
		return ERROR;
	}
	data -> Month = month;

	char* daystr = strtok(NULL, DATE_SP);
	if (daystr == NULL){
		return ERROR;
	}
	int day = atoi(daystr);
	if (day == 0){
		return ERROR;
	}
	data -> Day = day;


	// parse time
	if (times == NULL){
		return ERROR;
	}
	char* hourstr = strtok(times, TIME_SP);
	if (hourstr == NULL){
		return ERROR;
	}
	int hour = atoi(hourstr);
	data -> Hours = hour;

	char* minutestr = strtok(NULL, TIME_SP);
	if (minutestr == NULL){
		return ERROR;
	}
	int minute = atoi(minutestr);
	data -> Minutes = minute;

	char* secondstr = strtok(NULL, TIME_SP);
	if (secondstr == NULL){
		return ERROR;
	}
	int second = atoi(secondstr);
	data -> Seconds = second;

	return SUCCESS;
}

ErrorStatus Post_Login(UART_HandleTypeDef* uart, CommHandle_t* hcomm)
{
	//http://192.168.47.157:5276/api/device/login?deviceUID=123
	char url[256];
	int len = strlen(hcomm -> HttpDeviceUrl);
	memcpy(url, hcomm -> HttpDeviceUrl, len);
	url[len++] = '/';
	memcpy(url + len, LOGIN, strlen(LOGIN));
	len += strlen(LOGIN);
	url[len++] = '?';
	memcpy(url + len, DEVICEID_KEY, strlen(DEVICEID_KEY));
	len += strlen(DEVICEID_KEY);
	url[len++] = '=';
	memcpy(url + len, hcomm ->DeviceId , DEVICE_UID_LEN);
	len += DEVICE_UID_LEN;
	url[len++] = '\0';

	HAL_StatusTypeDef status = ESP_Http_Post(uart, url);
	if (status == HAL_OK){
		return SUCCESS;
	}
	else{
		return ERROR;
	}
}

ErrorStatus Post_Logout(UART_HandleTypeDef* uart, CommHandle_t* hcomm)
{
	//http://192.168.47.157:5276/api/device/logout?deviceUID=123
	char url[256];
	int len = strlen(hcomm -> HttpDeviceUrl);
	memcpy(url, hcomm -> HttpDeviceUrl, len);
	url[len++] = '/';
	memcpy(url + len, LOGOUT, strlen(LOGOUT));
	len += strlen(LOGOUT);
	url[len++] = '?';
	memcpy(url + len, DEVICEID_KEY, strlen(DEVICEID_KEY));
	len += strlen(DEVICEID_KEY);
	url[len++] = '=';
	memcpy(url + len, hcomm ->DeviceId , DEVICE_UID_LEN);
	len += DEVICE_UID_LEN;
	url[len++] = '\0';

	HAL_StatusTypeDef status = ESP_Http_Post(uart, url);
	if (status == HAL_OK){
		return SUCCESS;
	}
	else{
		return ERROR;
	}
}

ErrorStatus Get_CurrentTime(UART_HandleTypeDef* uart, CommHandle_t* hcomm, DateTime_t* dt)
{
	if (dt == NULL){
		return ERROR;
	}
	//'http://0.0.0.0:5276/api/device/timecurrent?DeviceUID=123'
	char url[256];
	int len = strlen(hcomm -> HttpDeviceUrl);
	memcpy(url, hcomm -> HttpDeviceUrl, len);
	url[len++] = '/';
	memcpy(url + len, CURRENT_TIME, strlen(CURRENT_TIME));
	len += strlen(CURRENT_TIME);
	url[len++] = '?';
	memcpy(url + len, DEVICEID_KEY, strlen(DEVICEID_KEY));
	len += strlen(DEVICEID_KEY);
	url[len++] = '=';
	memcpy(url + len, hcomm ->DeviceId , DEVICE_UID_LEN);
	len += DEVICE_UID_LEN;
	url[len++] = '\0';

	HAL_StatusTypeDef status = ESP_Http_Get(uart, url);
	if (status == HAL_OK){
		return SUCCESS;
	}
	else{
		return ERROR;
	}
}

ErrorStatus MQTT_Login(UART_HandleTypeDef* uart, CommHandle_t* hcomm)
{
	return SUCCESS;
}

ErrorStatus Wait_Response(UART_HandleTypeDef* uart, CommHandle_t* hcomm, char* pattern)
{
	return SUCCESS;
}

ErrorStatus Build_WeatherReportQuery(char* str, CommHandle_t* hcomm, uint8_t tempC, uint8_t humidity, char* deviceUID)
{
	//http://0.0.0.0:5276/api/weather?TemperatureC=1&Humidity=2&DeviceUID=3
	if (str == NULL || hcomm == NULL){
		return ERROR;
	}

	tempC = tempC > 99 ? 99 : tempC;
	humidity = humidity > 99 ? 99 : humidity;

	int len = 0;
	memcpy(str, hcomm -> HttpWeatherUrl, strlen(hcomm -> HttpWeatherUrl));
	len += strlen(hcomm -> HttpWeatherUrl);

	str[len++] = '?';

	memcpy(str + len, TEMPC, strlen(TEMPC));
	len += strlen(TEMPC);
	str[len++] = '=';
	str[len++] = tempC / 10 + '0';
	str[len++] = tempC % 10 + '0';

	str[len++] = '&';

	memcpy(str+len, HUMIDITY, strlen(HUMIDITY));
	len += strlen(HUMIDITY);
	str[len++] = '=';
	str[len++] = humidity / 10 + '0';
	str[len++] = humidity % 10 + '0';

	str[len++] = '&';

	memcpy(str+len, DEVICEID_KEY, strlen(DEVICEID_KEY));
	len += strlen(DEVICEID_KEY);
	str[len++] = '=';

	memcpy(str+len, deviceUID, DEVICE_UID_LEN);
	len+= DEVICE_UID_LEN;
	str[len++] = '\0';

	return SUCCESS;
}
