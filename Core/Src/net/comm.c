/*
 * http_comm.c
 *
 *  Created on: Jul 6, 2024
 *      Author: hung
 */

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#include "stm32f4xx.h"
#include "comm.h"
#include "utils.h"
#include "lwesp/lwesp_netconn.h"
#include "lwesp/lwesp_pbuf.h"

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

#define HTTP_REQ_BUFFER_SIZE 384
#define HTTP_RESP_BUFFER_SIZE 512
#define HTTP_DEFAULT_PORT 80
#define HTTP_TIMEOUT_MS 5000

typedef struct {
	char host[64];
	char path[192];
	uint16_t port;
} HttpUrlParts;

static ErrorStatus Http_Parse_Url(const char* url, HttpUrlParts* parts);
static const char* Http_Find_Body(char* response);
static ErrorStatus Http_Request(const char* method, const char* url, const char* body, size_t body_len,
		char* response, size_t response_len, const char** out_body);

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

static ErrorStatus Http_Parse_Url(const char* url, HttpUrlParts* parts)
{
	if (url == NULL || parts == NULL) {
		return ERROR;
	}

	const char* scheme = "http://";
	size_t scheme_len = strlen(scheme);
	if (strncmp(url, scheme, scheme_len) != 0) {
		return ERROR;
	}

	const char* host_start = url + scheme_len;
	const char* path_start = strchr(host_start, '/');
	const char* host_end = path_start != NULL ? path_start : (url + strlen(url));
	const char* port_sep = memchr(host_start, ':', (size_t)(host_end - host_start));

	size_t host_len = port_sep != NULL ? (size_t)(port_sep - host_start) : (size_t)(host_end - host_start);
	if (host_len == 0 || host_len >= sizeof(parts->host)) {
		return ERROR;
	}
	memcpy(parts->host, host_start, host_len);
	parts->host[host_len] = '\0';

	if (port_sep != NULL) {
		parts->port = (uint16_t)atoi(port_sep + 1);
		if (parts->port == 0) {
			return ERROR;
		}
	}
	else {
		parts->port = HTTP_DEFAULT_PORT;
	}

	if (path_start != NULL) {
		size_t path_len = strlen(path_start);
		if (path_len >= sizeof(parts->path)) {
			return ERROR;
		}
		memcpy(parts->path, path_start, path_len + 1);
	}
	else {
		memcpy(parts->path, "/", 2);
	}

	return SUCCESS;
}

static const char* Http_Find_Body(char* response)
{
	if (response == NULL) {
		return NULL;
	}
	char* body = strstr(response, "\r\n\r\n");
	if (body == NULL) {
		return response;
	}
	return body + 4;
}

static ErrorStatus Http_Request(const char* method, const char* url, const char* body, size_t body_len,
		char* response, size_t response_len, const char** out_body)
{
	if (method == NULL || url == NULL || response == NULL || response_len == 0) {
		return ERROR;
	}

	HttpUrlParts parts;
	if (Http_Parse_Url(url, &parts) != SUCCESS) {
		return ERROR;
	}

	lwesp_netconn_p conn = lwesp_netconn_new(LWESP_NETCONN_TYPE_TCP);
	if (conn == NULL) {
		return ERROR;
	}

	lwesp_netconn_set_receive_timeout(conn, HTTP_TIMEOUT_MS);
	if (lwesp_netconn_connect(conn, parts.host, parts.port) != lwespOK) {
		lwesp_netconn_delete(conn);
		return ERROR;
	}

	char request[HTTP_REQ_BUFFER_SIZE];
	int req_len = snprintf(request, sizeof(request),
			"%s %s HTTP/1.1\r\n"
			"Host: %s\r\n"
			"Connection: close\r\n"
			"Content-Length: %lu\r\n"
			"\r\n",
			method, parts.path, parts.host, (unsigned long)body_len);
	if (req_len <= 0 || (size_t)req_len >= sizeof(request)) {
		lwesp_netconn_close(conn);
		lwesp_netconn_delete(conn);
		return ERROR;
	}

	if (lwesp_netconn_write_ex(conn, request, (size_t)req_len, LWESP_NETCONN_FLAG_FLUSH) != lwespOK) {
		lwesp_netconn_close(conn);
		lwesp_netconn_delete(conn);
		return ERROR;
	}

	if (body != NULL && body_len > 0) {
		if (lwesp_netconn_write_ex(conn, body, body_len, LWESP_NETCONN_FLAG_FLUSH) != lwespOK) {
			lwesp_netconn_close(conn);
			lwesp_netconn_delete(conn);
			return ERROR;
		}
	}

	size_t offset = 0;
	while (offset + 1 < response_len) {
		lwesp_pbuf_p pbuf = NULL;
		lwespr_t res = lwesp_netconn_receive(conn, &pbuf);
		if (res != lwespOK) {
			if (pbuf != NULL) {
				lwesp_pbuf_free(pbuf);
			}
			break;
		}

		size_t len = lwesp_pbuf_length(pbuf, 1);
		size_t copy_len = len;
		if (copy_len > (response_len - 1 - offset)) {
			copy_len = response_len - 1 - offset;
		}
		if (copy_len > 0) {
			lwesp_pbuf_copy(pbuf, response + offset, copy_len, 0);
			offset += copy_len;
		}
		lwesp_pbuf_free(pbuf);
	}
	response[offset] = '\0';

	lwesp_netconn_close(conn);
	lwesp_netconn_delete(conn);

	if (offset == 0) {
		return ERROR;
	}

	if (out_body != NULL) {
		*out_body = Http_Find_Body(response);
	}
	return SUCCESS;
}

ErrorStatus Post_Login(CommHandle_t* hcomm)
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

	return Http_Post_Url(url, POST_SUCCESS);
}

ErrorStatus Post_Logout(CommHandle_t* hcomm)
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

	return Http_Post_Url(url, POST_SUCCESS);
}

ErrorStatus Get_CurrentTime(CommHandle_t* hcomm, DateTime_t* dt)
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

	char response[HTTP_RESP_BUFFER_SIZE];
	const char* body = NULL;
	if (Http_Request("GET", url, NULL, 0, response, sizeof(response), &body) != SUCCESS) {
		return ERROR;
	}

	if (body == NULL) {
		return ERROR;
	}

	char* dt_res = strchr((char*)body, KeyValueSeperater);
	if (dt_res == NULL) {
		return ERROR;
	}
	return Try_Parse_Time(dt_res + 1, dt);
}

ErrorStatus Http_Post_Url(const char* url, const char* expect)
{
	char response[HTTP_RESP_BUFFER_SIZE];
	const char* body = NULL;
	if (Http_Request("POST", url, NULL, 0, response, sizeof(response), &body) != SUCCESS) {
		return ERROR;
	}

	if (expect != NULL && body != NULL && strstr(body, expect) != NULL) {
		return SUCCESS;
	}
	return ERROR;
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
