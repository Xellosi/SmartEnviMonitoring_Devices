/*
 * esp32at.c
 *
 *  Created on: Jul 6, 2024
 *      Author: hung
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "esp32at.h"

#define ESP32AT_LINE_BUFFER_SIZE 192
#define ESP32AT_RX_BUFFER_SIZE 4096
#define ESP32AT_POLL_DELAY_MS 20
#define ESP32AT_HTTPCLIENT_CMD_SIZE 256
#define ESP32AT_HTTPCLIENT_TRANSPORT_HTTP 1u
#define ESP32AT_HTTPCLIENT_CONTENT_TYPE_URLENCODED 0u
#define ESP32AT_HTTPCLIENT_FMT_BASIC "AT+HTTPCLIENT=%u,%u,\"%s\",\"%s\",\"%s\",%u"
#define ESP32AT_HTTPCLIENT_FMT_BODY "AT+HTTPCLIENT=%u,%u,\"%s\",\"%s\",\"%s\",%u,\"%s\""

static UART_HandleTypeDef* esp_uart = NULL;

static SemaphoreHandle_t esp32at_resp_sem = NULL;
static volatile int8_t esp32at_resp_status = 0;
static volatile bool esp32at_waiting = false;
static char esp32at_line_buf[ESP32AT_LINE_BUFFER_SIZE];
static size_t esp32at_line_len = 0;

static char esp32at_rx_buf[ESP32AT_RX_BUFFER_SIZE];
static size_t esp32at_rx_len = 0;

static void ESP32AT_LogRxBuffer(const char* context);
static size_t ESP32AT_ParseHttpClientBody(char* out, size_t out_len, int* status_code);
static void ESP32AT_HandleErrorCodeLine(const char* line);
static bool ESP32AT_SplitUrl(const char* url, char* host, size_t host_len, char* path, size_t path_len);
static bool ESP32AT_GetWifiState(int* out_state, uint32_t timeout_ms);
static bool ESP32AT_WifiConnect(const char* ssid, const char* pass, uint32_t timeout_ms);
static void ESP32AT_LogWifiInfo(uint32_t timeout_ms);

static void ESP32AT_ResetRxBuffer(void)
{
	esp32at_rx_len = 0;
	esp32at_rx_buf[0] = '\0';
}

static void ESP32AT_AppendRx(const uint8_t* data, size_t len)
{
	if (data == NULL || len == 0) {
		return;
	}

	if (len >= (ESP32AT_RX_BUFFER_SIZE - 1)) {
		memcpy(esp32at_rx_buf, data + (len - (ESP32AT_RX_BUFFER_SIZE - 1)), ESP32AT_RX_BUFFER_SIZE - 1);
		esp32at_rx_len = ESP32AT_RX_BUFFER_SIZE - 1;
		esp32at_rx_buf[esp32at_rx_len] = '\0';
		return;
	}

	if (esp32at_rx_len + len >= (ESP32AT_RX_BUFFER_SIZE - 1)) {
		size_t keep = (ESP32AT_RX_BUFFER_SIZE - 1) - len;
		if (esp32at_rx_len > keep) {
			memmove(esp32at_rx_buf, esp32at_rx_buf + (esp32at_rx_len - keep), keep);
			esp32at_rx_len = keep;
		}
	}

	memcpy(esp32at_rx_buf + esp32at_rx_len, data, len);
	esp32at_rx_len += len;
	esp32at_rx_buf[esp32at_rx_len] = '\0';
}

static bool ESP32AT_BufferContains(const char* token)
{
	if (token == NULL || token[0] == '\0') {
		return false;
	}
	return strstr(esp32at_rx_buf, token) != NULL;
}

static bool ESP32AT_WaitForAnyToken(const char* const* tokens, size_t token_count, uint32_t timeout_ms,
		size_t* hit_index)
{
	TickType_t start = xTaskGetTickCount();
	TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);

	if (hit_index != NULL) {
		*hit_index = 0;
	}

	for (;;) {
		for (size_t i = 0; i < token_count; ++i) {
			if (ESP32AT_BufferContains(tokens[i])) {
				if (hit_index != NULL) {
					*hit_index = i;
				}
				return true;
			}
		}
		if (timeout_ms == 0) {
			break;
		}
		if ((xTaskGetTickCount() - start) >= timeout_ticks) {
			break;
		}
		vTaskDelay(pdMS_TO_TICKS(ESP32AT_POLL_DELAY_MS));
	}
	return false;
}

static bool ESP32AT_WaitForToken(const char* token, uint32_t timeout_ms)
{
	const char* tokens[] = {token};
	return ESP32AT_WaitForAnyToken(tokens, 1, timeout_ms, NULL);
}

static void ESP32AT_HandleLine(const char* line)
{
	if (line == NULL) {
		return;
	}
	ESP32AT_HandleErrorCodeLine(line);
	if (!esp32at_waiting) {
		return;
	}
	if (strcmp(line, "OK") == 0) {
		esp32at_resp_status = 1;
		esp32at_waiting = false;
		if (esp32at_resp_sem != NULL) {
			xSemaphoreGive(esp32at_resp_sem);
		}
		return;
	}
	if (strcmp(line, "ERROR") == 0 || strcmp(line, "FAIL") == 0) {
		esp32at_resp_status = -1;
		esp32at_waiting = false;
		if (esp32at_resp_sem != NULL) {
			xSemaphoreGive(esp32at_resp_sem);
		}
	}
}

static void ESP32AT_ProcessInput(const uint8_t* data, size_t len)
{
	if (data == NULL || len == 0) {
		return;
	}
	for (size_t i = 0; i < len; ++i) {
		char ch = (char)data[i];
		if (ch == '\r') {
			continue;
		}
		if (ch == '\n') {
			if (esp32at_line_len > 0) {
				esp32at_line_buf[esp32at_line_len] = '\0';
				ESP32AT_HandleLine(esp32at_line_buf);
				esp32at_line_len = 0;
			}
			continue;
		}
		if (esp32at_line_len + 1 < sizeof(esp32at_line_buf)) {
			esp32at_line_buf[esp32at_line_len++] = ch;
		} else {
			esp32at_line_len = 0;
		}
	}
}

static void ESP32AT_PrintData(const char* tag, const uint8_t* data, size_t len)
{
	if (tag == NULL || data == NULL || len == 0) {
		return;
	}
	printf("%s", tag);
	printf("%.*s", (int)len, (const char*)data);
	if (data[len - 1] != '\n' && data[len - 1] != '\r') {
		printf("\n");
	}
}

static size_t ESP32AT_ParseHttpClientBody(char* out, size_t out_len, int* status_code)
{
	if (out == NULL || out_len == 0) {
		return 0;
	}

	const char* tag = strstr(esp32at_rx_buf, "+HTTPCLIENT:");
	if (tag == NULL) {
		ESP32AT_LogRxBuffer("HTTPCLIENT missing");
		return 0;
	}

	const char* p = strchr(tag, ':');
	if (p == NULL) {
		ESP32AT_LogRxBuffer("HTTPCLIENT parse failed");
		return 0;
	}
	p++;
	while (*p == ' ') {
		p++;
	}

	const char* line_end = strchr(p, '\n');
	if (line_end == NULL) {
		line_end = esp32at_rx_buf + esp32at_rx_len;
	}
	const char* line_end_trim = line_end;
	if (line_end_trim > p && *(line_end_trim - 1) == '\r') {
		line_end_trim--;
	}

	size_t line_len = (size_t)(line_end_trim - p);
	const char* comma1 = memchr(p, ',', line_len);
	if (comma1 == NULL) {
		ESP32AT_LogRxBuffer("HTTPCLIENT parse failed");
		return 0;
	}

	char* end = NULL;
	long first = strtol(p, &end, 10);
	if (end == p || end > comma1) {
		ESP32AT_LogRxBuffer("HTTPCLIENT parse failed");
		return 0;
	}

	const char* rest = comma1 + 1;
	while (rest < line_end_trim && *rest == ' ') {
		rest++;
	}

	const char* comma2 = memchr(rest, ',', (size_t)(line_end_trim - rest));
	if (comma2 != NULL) {
		long data_len = strtol(rest, &end, 10);
		if (end == rest || end > comma2) {
			if (status_code != NULL) {
				*status_code = 0;
			}
			size_t inline_len = (size_t)(line_end_trim - rest);
			if (inline_len >= out_len) {
				inline_len = out_len - 1;
			}
			memcpy(out, rest, inline_len);
			out[inline_len] = '\0';
			return inline_len;
		}
		(void)data_len;
		if (status_code != NULL) {
			*status_code = (int)first;
		}
		const char* data_start = comma2 + 1;
		while (data_start < line_end_trim && *data_start == ' ') {
			data_start++;
		}
		size_t inline_len = (size_t)(line_end_trim - data_start);
		if (inline_len >= out_len) {
			inline_len = out_len - 1;
		}
		memcpy(out, data_start, inline_len);
		out[inline_len] = '\0';
		return inline_len;
	}

	bool rest_is_digits = rest < line_end_trim;
	for (const char* t = rest; t < line_end_trim; ++t) {
		if (*t < '0' || *t > '9') {
			rest_is_digits = false;
			break;
		}
	}

	if (!rest_is_digits) {
		if (status_code != NULL) {
			*status_code = 0;
		}
		size_t inline_len = (size_t)(line_end_trim - rest);
		if (inline_len >= out_len) {
			inline_len = out_len - 1;
		}
		memcpy(out, rest, inline_len);
		out[inline_len] = '\0';
		return inline_len;
	}

	long data_len = strtol(rest, &end, 10);
	if (end == rest) {
		data_len = -1;
	}
	if (status_code != NULL) {
		*status_code = (int)first;
	}

	const char* buffer_end = esp32at_rx_buf + esp32at_rx_len;
	const char* body = line_end < buffer_end ? line_end + 1 : line_end;
	if (body < buffer_end && *body == '\r') {
		body++;
	}

	size_t available = body < buffer_end ? (size_t)(buffer_end - body) : 0;
	size_t copy_len = 0;
	if (data_len >= 0 && (size_t)data_len <= available) {
		copy_len = (size_t)data_len;
	} else {
		const char* ok = strstr(body, "\r\nOK");
		if (ok == NULL) {
			ok = strstr(body, "\nOK");
		}
		if (ok != NULL) {
			available = (size_t)(ok - body);
		}
		copy_len = available;
	}

	if (copy_len >= out_len) {
		copy_len = out_len - 1;
	}
	memcpy(out, body, copy_len);
	out[copy_len] = '\0';
	return copy_len;
}

static void ESP32AT_LogRxBuffer(const char* context)
{
	if (context != NULL && context[0] != '\0') {
		printf("[ESP32 RX BUF] %s\n", context);
	}
	if (esp32at_rx_len == 0) {
		printf("[ESP32 RX BUF] <empty>\n");
		return;
	}
	ESP32AT_PrintData("[ESP32 RX BUF] ", (const uint8_t*)esp32at_rx_buf, esp32at_rx_len);
}

static void ESP32AT_HandleErrorCodeLine(const char* line)
{
	const char* prefix = "ERR CODE:";
	size_t prefix_len = strlen(prefix);
	if (strncmp(line, prefix, prefix_len) != 0) {
		return;
	}

	const char* p = line + prefix_len;
	while (*p == ' ') {
		p++;
	}
	unsigned long code = strtoul(p, NULL, 16);
	if (code == 0) {
		return;
	}

	unsigned long http_code = code & 0xFFFFu;
	if ((http_code & 0xF000u) == 0x7000u) {
		if (http_code == 0x7000u) {
			printf("[ESP32 ERR] 0x%08lx HTTP connect failed\n", code);
			return;
		}
		if (http_code >= 0x7190u && http_code <= 0x719Fu) {
			printf("[ESP32 ERR] 0x%08lx HTTP status %lu\n", code,
					400ul + (http_code - 0x7190u));
			return;
		}
		if (http_code >= 0x71A0u && http_code <= 0x71A1u) {
			printf("[ESP32 ERR] 0x%08lx HTTP status %lu\n", code,
					416ul + (http_code - 0x71A0u));
			return;
		}
		if (http_code >= 0x71F4u && http_code <= 0x71F9u) {
			printf("[ESP32 ERR] 0x%08lx HTTP status %lu\n", code,
					500ul + (http_code - 0x71F4u));
			return;
		}
		printf("[ESP32 ERR] 0x%08lx HTTP error 0x%04lx\n", code, http_code);
		return;
	}

	printf("[ESP32 ERR] 0x%08lx\n", code);
}

static bool ESP32AT_SplitUrl(const char* url, char* host, size_t host_len, char* path, size_t path_len)
{
	if (url == NULL || host == NULL || path == NULL || host_len == 0 || path_len == 0) {
		return false;
	}

	const char* scheme = "http://";
	size_t scheme_len = strlen(scheme);
	if (strncmp(url, scheme, scheme_len) != 0) {
		return false;
	}

	const char* host_start = url + scheme_len;
	const char* path_start = strchr(host_start, '/');
	const char* host_end = path_start != NULL ? path_start : (url + strlen(url));
	const char* port_sep = memchr(host_start, ':', (size_t)(host_end - host_start));

	if (port_sep != NULL) {
		int port = atoi(port_sep + 1);
		if (port != 0 && port != 80) {
			return false;
		}
	}

	size_t host_copy_len = port_sep != NULL ? (size_t)(port_sep - host_start) : (size_t)(host_end - host_start);
	if (host_copy_len == 0 || host_copy_len >= host_len) {
		return false;
	}
	memcpy(host, host_start, host_copy_len);
	host[host_copy_len] = '\0';

	if (path_start != NULL) {
		size_t path_copy_len = strlen(path_start);
		if (path_copy_len >= path_len) {
			return false;
		}
		memcpy(path, path_start, path_copy_len + 1);
	} else {
		memcpy(path, "/", 2);
	}

	return true;
}

static bool ESP32AT_GetWifiState(int* out_state, uint32_t timeout_ms)
{
	if (!ESP32AT_SendCommandWait("AT+CWSTATE?", timeout_ms)) {
		ESP32AT_LogRxBuffer("CWSTATE failed");
		return false;
	}

	const char* tag = strstr(esp32at_rx_buf, "+CWSTATE:");
	if (tag == NULL) {
		ESP32AT_LogRxBuffer("CWSTATE missing");
		return false;
	}

	const char* p = strchr(tag, ':');
	if (p == NULL) {
		ESP32AT_LogRxBuffer("CWSTATE parse failed");
		return false;
	}
	int state = atoi(p + 1);
	printf("[ESP32] CWSTATE=%d\n", state);
	if (out_state != NULL) {
		*out_state = state;
	}
	return true;
}

static bool ESP32AT_WifiConnect(const char* ssid, const char* pass, uint32_t timeout_ms)
{
	printf("[ESP32] WIFI reconnecting\n");
	if (ssid == NULL || ssid[0] == '\0') {
		return ESP32AT_SendCommandWait("AT+CWJAP", timeout_ms);
	}
	const char* pwd = pass != NULL ? pass : "";
	char cmd[160];
	int len = snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"", ssid, pwd);
	if (len <= 0 || (size_t)len >= sizeof(cmd)) {
		return false;
	}
	return ESP32AT_SendCommandWait(cmd, timeout_ms);
}

static void ESP32AT_LogWifiInfo(uint32_t timeout_ms)
{
	printf("[ESP32] WIFI info\n");
	(void)ESP32AT_SendCommandWait("AT+CWSTATE?", timeout_ms);
	(void)ESP32AT_SendCommandWait("AT+CWJAP?", timeout_ms);
	(void)ESP32AT_SendCommandWait("AT+CIPSTA?", timeout_ms);
}

void ESP32AT_SetUart(UART_HandleTypeDef* uart)
{
	esp_uart = uart;
}

void ESP32AT_Input(const uint8_t* data, size_t len)
{
	if (data == NULL || len == 0) {
		return;
	}
	ESP32AT_PrintData("[ESP32 RX] ", data, len);
	ESP32AT_AppendRx(data, len);
	ESP32AT_ProcessInput(data, len);
}

size_t ESP32AT_SendRaw(const uint8_t* data, size_t len)
{
	if (esp_uart == NULL || data == NULL || len == 0) {
		return 0;
	}
	if (HAL_UART_Transmit(esp_uart, (const uint8_t*)data, len, HAL_MAX_DELAY) != HAL_OK) {
		return 0;
	}
	ESP32AT_PrintData("[ESP32 TX] ", data, len);
	return len;
}

size_t ESP32AT_SendCommand(const char* cmd)
{
	if (cmd == NULL) {
		return 0;
	}
	size_t len = strlen(cmd);
	size_t total_len = len + 2;
	uint8_t* buffer = (uint8_t*)pvPortMalloc(total_len);
	if (buffer == NULL) {
		return 0;
	}
	if (len > 0) {
		memcpy(buffer, cmd, len);
	}
	buffer[len] = '\r';
	buffer[len + 1] = '\n';
	size_t sent = ESP32AT_SendRaw(buffer, total_len);
	vPortFree(buffer);
	return sent;
}

bool ESP32AT_SendCommandWait(const char* cmd, uint32_t timeout_ms)
{
	if (cmd == NULL) {
		return false;
	}
	if (esp32at_resp_sem == NULL) {
		esp32at_resp_sem = xSemaphoreCreateBinary();
		if (esp32at_resp_sem == NULL) {
			return false;
		}
	}
	(void)xSemaphoreTake(esp32at_resp_sem, 0);
	esp32at_resp_status = 0;
	esp32at_waiting = true;
	esp32at_line_len = 0;
	ESP32AT_ResetRxBuffer();

	if (ESP32AT_SendCommand(cmd) == 0) {
		esp32at_waiting = false;
		return false;
	}

	TickType_t ticks = pdMS_TO_TICKS(timeout_ms);
	if (xSemaphoreTake(esp32at_resp_sem, ticks) != pdTRUE) {
		esp32at_waiting = false;
		return false;
	}
	return esp32at_resp_status > 0;
}

bool ESP32AT_Init(const char* wifi_ssid, const char* wifi_pass, uint32_t timeout_ms)
{
	if (!ESP32AT_SendCommandWait("AT", timeout_ms)) {
		return false;
	}
	(void)ESP32AT_SendCommandWait("ATE0", timeout_ms);
	(void)ESP32AT_SendCommandWait("AT+SYSLOG=1", timeout_ms);
	(void)ESP32AT_SendCommandWait("AT+CWINIT=1", timeout_ms);
	(void)ESP32AT_SendCommandWait("AT+CWMODE=1,1", timeout_ms);
	if (!ESP32AT_SendCommandWait("AT+CIPMUX=0", timeout_ms)) {
		return false;
	}
	(void)ESP32AT_SendCommandWait("AT+CIPDINFO=0", timeout_ms);
	(void)ESP32AT_SendCommandWait("AT+CIPRECVMODE=0", timeout_ms);
	ESP32AT_LogWifiInfo(timeout_ms);
	int state = -1;
	if (!ESP32AT_GetWifiState(&state, timeout_ms)) {
		return false;
	}
	if (state != 2) {
		if (!ESP32AT_WifiConnect(wifi_ssid, wifi_pass, timeout_ms)) {
			return false;
		}
		ESP32AT_LogWifiInfo(timeout_ms);
		if (!ESP32AT_GetWifiState(&state, timeout_ms)) {
			return false;
		}
	}
	if (state != 2) {
		return false;
	}
	return true;
}

size_t ESP32AT_HttpClientRequest(Esp32AtHttpMethod method, const char* url, const char* body, size_t body_len,
		char* response, size_t response_len, int* status_code, uint32_t timeout_ms)
{
	if (url == NULL || response == NULL || response_len == 0) {
		return 0;
	}

	(void)body_len;
	const char* payload = body != NULL ? body : "";
	char host_buf[64];
	char path_buf[192];
	const char* host = "";
	const char* path = "";

	if (ESP32AT_SplitUrl(url, host_buf, sizeof(host_buf), path_buf, sizeof(path_buf))) {
		host = host_buf;
		path = path_buf;
	}

	char cmd[ESP32AT_HTTPCLIENT_CMD_SIZE];
	int len = 0;
	if (method == ESP32AT_HTTP_METHOD_POST) {
		len = snprintf(cmd, sizeof(cmd), ESP32AT_HTTPCLIENT_FMT_BODY, (unsigned)method,
				(unsigned)ESP32AT_HTTPCLIENT_CONTENT_TYPE_URLENCODED, url, host, path,
				(unsigned)ESP32AT_HTTPCLIENT_TRANSPORT_HTTP, payload);
	} else {
		len = snprintf(cmd, sizeof(cmd), ESP32AT_HTTPCLIENT_FMT_BASIC, (unsigned)method,
				(unsigned)ESP32AT_HTTPCLIENT_CONTENT_TYPE_URLENCODED, url, host, path,
				(unsigned)ESP32AT_HTTPCLIENT_TRANSPORT_HTTP);
	}
	if (len <= 0 || (size_t)len >= sizeof(cmd)) {
		return 0;
	}

	if (!ESP32AT_SendCommandWait(cmd, timeout_ms)) {
		ESP32AT_LogRxBuffer("HTTPCLIENT failed");
		return 0;
	}

	size_t copied = ESP32AT_ParseHttpClientBody(response, response_len, status_code);
	if (copied == 0) {
		ESP32AT_LogRxBuffer("HTTPCLIENT empty response");
		return 0;
	}

	if (status_code != NULL && *status_code > 0 && (*status_code < 200 || *status_code >= 300)) {
		char ctx[48];
		snprintf(ctx, sizeof(ctx), "HTTPCLIENT status %d", *status_code);
		ESP32AT_LogRxBuffer(ctx);
	}

	return copied;
}

bool ESP32AT_TcpConnect(const char* host, uint16_t port, uint32_t timeout_ms)
{
	if (host == NULL || port == 0) {
		return false;
	}
	char cmd[128];
	int len = snprintf(cmd, sizeof(cmd), "AT+CIPSTART=\"TCP\",\"%s\",%u", host, (unsigned)port);
	if (len <= 0 || (size_t)len >= sizeof(cmd)) {
		return false;
	}
	ESP32AT_ResetRxBuffer();
	if (ESP32AT_SendCommand(cmd) == 0) {
		return false;
	}
	const char* tokens[] = {"OK", "CONNECT", "ALREADY CONNECTED", "ERROR", "FAIL"};
	size_t hit = 0;
	if (!ESP32AT_WaitForAnyToken(tokens, 5, timeout_ms, &hit)) {
		ESP32AT_LogRxBuffer("CIPSTART timeout");
		return false;
	}
	if (hit >= 3) {
		ESP32AT_LogRxBuffer("CIPSTART error");
		return false;
	}
	return hit < 3;
}

bool ESP32AT_TcpSend(const uint8_t* data, size_t len, uint32_t timeout_ms)
{
	if (data == NULL || len == 0) {
		return false;
	}
	char cmd[64];
	int cmd_len = snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%lu", (unsigned long)len);
	if (cmd_len <= 0 || (size_t)cmd_len >= sizeof(cmd)) {
		return false;
	}
	ESP32AT_ResetRxBuffer();
	if (ESP32AT_SendCommand(cmd) == 0) {
		return false;
	}
	if (!ESP32AT_WaitForToken(">", timeout_ms)) {
		return false;
	}
	if (ESP32AT_SendRaw(data, len) != len) {
		return false;
	}
	const char* tokens[] = {"SEND OK", "SEND FAIL", "ERROR", "FAIL"};
	size_t hit = 0;
	if (!ESP32AT_WaitForAnyToken(tokens, 4, timeout_ms, &hit)) {
		return false;
	}
	return hit == 0;
}

static bool ESP32AT_ParseIpd(size_t start, size_t* data_pos, size_t* data_len, size_t* next_pos)
{
	if (start >= esp32at_rx_len) {
		return false;
	}
	const char* base = esp32at_rx_buf + start;
	const char* ipd = strstr(base, "+IPD,");
	if (ipd == NULL) {
		return false;
	}
	const char* len_start = ipd + 5;
	char* len_end = NULL;
	long parsed = strtol(len_start, &len_end, 10);
	if (len_end == NULL || *len_end != ':' || parsed <= 0) {
		return false;
	}
	size_t payload_len = (size_t)parsed;
	size_t payload_pos = (size_t)(len_end - esp32at_rx_buf + 1);
	if (payload_pos + payload_len > esp32at_rx_len) {
		return false;
	}
	*data_pos = payload_pos;
	*data_len = payload_len;
	*next_pos = payload_pos + payload_len;
	return true;
}

size_t ESP32AT_TcpReceive(char* out, size_t out_len, uint32_t timeout_ms)
{
	if (out == NULL || out_len == 0) {
		return 0;
	}
	size_t out_pos = 0;
	size_t parse_pos = 0;
	TickType_t start = xTaskGetTickCount();
	TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);

	while (out_pos + 1 < out_len) {
		size_t data_pos = 0;
		size_t data_len = 0;
		size_t next_pos = 0;
		if (ESP32AT_ParseIpd(parse_pos, &data_pos, &data_len, &next_pos)) {
			size_t copy_len = data_len;
			if (copy_len > (out_len - 1 - out_pos)) {
				copy_len = out_len - 1 - out_pos;
			}
			memcpy(out + out_pos, esp32at_rx_buf + data_pos, copy_len);
			out_pos += copy_len;
			parse_pos = next_pos;
			continue;
		}
		if (ESP32AT_BufferContains("CLOSED") || ESP32AT_BufferContains("ERROR")
				|| ESP32AT_BufferContains("FAIL")) {
			break;
		}
		if (timeout_ms == 0) {
			break;
		}
		if ((xTaskGetTickCount() - start) >= timeout_ticks) {
			break;
		}
		vTaskDelay(pdMS_TO_TICKS(ESP32AT_POLL_DELAY_MS));
	}
	out[out_pos] = '\0';
	return out_pos;
}

bool ESP32AT_TcpClose(uint32_t timeout_ms)
{
	ESP32AT_ResetRxBuffer();
	if (ESP32AT_SendCommand("AT+CIPCLOSE") == 0) {
		return false;
	}
	const char* tokens[] = {"OK", "ERROR", "FAIL"};
	size_t hit = 0;
	if (!ESP32AT_WaitForAnyToken(tokens, 3, timeout_ms, &hit)) {
		return false;
	}
	return hit == 0;
}

bool ESP32AT_MqttConfig(const char* client_id, const char* user, const char* pass, uint16_t keep_alive)
{
	const char* safe_id = client_id != NULL ? client_id : "";
	const char* safe_user = user != NULL ? user : "";
	const char* safe_pass = pass != NULL ? pass : "";
	char cmd[192];
	int len = snprintf(cmd, sizeof(cmd),
			"AT+MQTTUSERCFG=0,1,\"%s\",\"%s\",\"%s\",0,0,\"\"",
			safe_id, safe_user, safe_pass);
	if (len <= 0 || (size_t)len >= sizeof(cmd)) {
		return false;
	}
	return ESP32AT_SendCommandWait(cmd, 2000u);
}

bool ESP32AT_MqttConnect(const char* host, uint16_t port, uint32_t timeout_ms)
{
	if (host == NULL || port == 0) {
		return false;
	}
	char cmd[160];
	int len = snprintf(cmd, sizeof(cmd), "AT+MQTTCONN=0,\"%s\",%u,0", host, (unsigned)port);
	if (len <= 0 || (size_t)len >= sizeof(cmd)) {
		return false;
	}
	return ESP32AT_SendCommandWait(cmd, timeout_ms);
}

bool ESP32AT_MqttSubscribe(const char* topic, uint8_t qos, uint32_t timeout_ms)
{
	if (topic == NULL) {
		return false;
	}
	char cmd[192];
	int len = snprintf(cmd, sizeof(cmd), "AT+MQTTSUB=0,\"%s\",%u", topic, (unsigned)qos);
	if (len <= 0 || (size_t)len >= sizeof(cmd)) {
		return false;
	}
	return ESP32AT_SendCommandWait(cmd, timeout_ms);
}

bool ESP32AT_MqttPublish(const char* topic, const char* payload, uint8_t qos, bool retain, uint32_t timeout_ms)
{
	if (topic == NULL || payload == NULL) {
		return false;
	}
	char cmd[256];
	int len = snprintf(cmd, sizeof(cmd), "AT+MQTTPUB=0,\"%s\",\"%s\",%u,%u",
			topic, payload, (unsigned)qos, retain ? 1u : 0u);
	if (len <= 0 || (size_t)len >= sizeof(cmd)) {
		return false;
	}
	return ESP32AT_SendCommandWait(cmd, timeout_ms);
}

bool ESP32AT_MqttDisconnect(uint32_t timeout_ms)
{
	return ESP32AT_SendCommandWait("AT+MQTTDISCONN=0", timeout_ms);
}
