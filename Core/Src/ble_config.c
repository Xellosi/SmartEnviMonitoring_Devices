/*
 * ble_config.c
 *
 *  Created on: Sep 18, 2025
 */

#include "ble_config.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "config_storage.h"
#include "esp32at.h"

#define BLE_LINE_BUFFER_SIZE 192
#define BLE_RESP_BUFFER_SIZE 64
#define BLE_AT_CMD_TIMEOUT_MS 2000u

#define BLE_AT_INIT_CMD "AT+BLEINIT=2"
#define BLE_AT_NAME_CMD "AT+BLENAME=\"SmartEnviCfg\""
#define BLE_AT_SECPARAM_CMD "AT+BLESECPARAM=1,3,16,3,3"
#define BLE_AT_CREATE_SERVICE_FMT "AT+BLEGATTSSRVCRE=\"%s\""
#define BLE_AT_CREATE_CHAR_FMT "AT+BLEGATTSCHAR=%lu,\"%s\",%u,%u,%u"
#define BLE_AT_START_SERVICE_FMT "AT+BLEGATTSSRVSTART=%lu"
#define BLE_AT_ADV_START_CMD "AT+BLEADVSTART"

#define BLE_AT_SETATTR_FMT "AT+BLEGATTSSETATTR=%lu,%u,\"%s\""
#define BLE_AT_NOTIFY_FMT "AT+BLEGATTSNTFY=%lu,%lu,%u,\"%s\""

#define BLE_GATTS_CHAR_PERM 0x02u
#define BLE_GATTS_CHAR_PROP 0x18u
#define BLE_GATTS_CHAR_MAX_LEN 32u

#define BLE_CHAR_HANDLE_FALLBACK 0x002Aul

static BleConfigApplyFn ble_apply_fn = NULL;
static char ble_line_buf[BLE_LINE_BUFFER_SIZE];
static size_t ble_line_len = 0;
static uint32_t ble_service_id = 0;
static bool ble_service_id_valid = false;
static uint32_t ble_last_conn_id = 0;
static uint32_t ble_char_handle = 0;
static volatile bool ble_wait_for_ip = false;

static void Ble_Process_Line(const char* line);
static bool Ble_Extract_Ip(const char* line, char* out_ip, size_t out_len);
static void Ble_Send_Command(const char* cmd);
static void Ble_Send_Response(bool ok);
static uint32_t Ble_Parse_First_Number(const char* line);
static uint32_t Ble_Parse_Last_Number(const char* line);
static void Ble_Update_WaitFlag(const char* line);

void Ble_Config_SetApplyFn(BleConfigApplyFn fn)
{
	ble_apply_fn = fn;
}

void Ble_Config_Init(void)
{
	char cmd[BLE_LINE_BUFFER_SIZE];

	Ble_Send_Command(BLE_AT_INIT_CMD);
	Ble_Send_Command(BLE_AT_NAME_CMD);
	Ble_Send_Command(BLE_AT_SECPARAM_CMD);

	snprintf(cmd, sizeof(cmd), BLE_AT_CREATE_SERVICE_FMT, BLE_CONFIG_SERVICE_UUID);
	Ble_Send_Command(cmd);

	uint32_t srv_id = ble_service_id_valid ? ble_service_id : 0;
	snprintf(cmd, sizeof(cmd), BLE_AT_CREATE_CHAR_FMT, (unsigned long)srv_id, BLE_CONFIG_CHAR_UUID,
			(unsigned)BLE_GATTS_CHAR_PERM, (unsigned)BLE_GATTS_CHAR_PROP, (unsigned)BLE_GATTS_CHAR_MAX_LEN);
	Ble_Send_Command(cmd);

	snprintf(cmd, sizeof(cmd), BLE_AT_START_SERVICE_FMT, (unsigned long)srv_id);
	Ble_Send_Command(cmd);
	Ble_Send_Command(BLE_AT_ADV_START_CMD);
}

void Ble_Config_HandleRx(const uint8_t* data, size_t len)
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
			if (ble_line_len > 0) {
				ble_line_buf[ble_line_len] = '\0';
				Ble_Process_Line(ble_line_buf);
				ble_line_len = 0;
			}
			continue;
		}
		if (ble_line_len + 1 < sizeof(ble_line_buf)) {
			ble_line_buf[ble_line_len++] = ch;
		} else {
			ble_line_len = 0;
		}
	}
}

bool Ble_Config_ShouldWaitIp(void)
{
	return ble_wait_for_ip;
}

static void Ble_Process_Line(const char* line)
{
	if (line == NULL) {
		return;
	}

	if (strstr(line, "+BLE") == NULL) {
		return;
	}

	Ble_Update_WaitFlag(line);

	if (strstr(line, "CONN") != NULL) {
		ble_last_conn_id = Ble_Parse_First_Number(line);
	}

	if (strstr(line, "GATTSSRVCRE") != NULL) {
		ble_service_id = Ble_Parse_Last_Number(line);
		ble_service_id_valid = true;
	}

	if (strstr(line, "GATTSCHAR") != NULL) {
		ble_char_handle = Ble_Parse_Last_Number(line);
	}

	char ip[CONFIG_STORAGE_MAX_IP_LEN];
	if (!Ble_Extract_Ip(line, ip, sizeof(ip))) {
		return;
	}

	if (ble_apply_fn != NULL) {
		bool ok = ConfigStorage_IsValidIp(ip);
		if (ok) {
			ble_apply_fn(ip, true, false);
			ble_wait_for_ip = false;
		}
		Ble_Send_Response(ok);
	}
}

static bool Ble_Extract_Ip(const char* line, char* out_ip, size_t out_len)
{
	if (line == NULL || out_ip == NULL || out_len == 0) {
		return false;
	}

	const char* p = line;
	while (*p != '\0') {
		if (!isdigit((unsigned char)*p)) {
			p++;
			continue;
		}
		char candidate[CONFIG_STORAGE_MAX_IP_LEN];
		size_t len = 0;
		const char* start = p;
		while (*p != '\0' && (isdigit((unsigned char)*p) || *p == '.')) {
			if (len + 1 < sizeof(candidate)) {
				candidate[len++] = *p;
			} else {
				break;
			}
			p++;
		}
		candidate[len] = '\0';
		if (ConfigStorage_IsValidIp(candidate)) {
			size_t copy_len = len < (out_len - 1) ? len : (out_len - 1);
			memcpy(out_ip, candidate, copy_len);
			out_ip[copy_len] = '\0';
			return true;
		}
		p = start + 1;
	}

	return false;
}

static void Ble_Send_Command(const char* cmd)
{
	(void)ESP32AT_SendCommandWait(cmd, BLE_AT_CMD_TIMEOUT_MS);
}

static void Ble_Send_Response(bool ok)
{
	char resp[BLE_RESP_BUFFER_SIZE];
	const char* text = ok ? "OK" : "ERR";
	uint32_t conn_id = ble_last_conn_id;
	uint32_t handle = ble_char_handle != 0 ? ble_char_handle : BLE_CHAR_HANDLE_FALLBACK;

	snprintf(resp, sizeof(resp), BLE_AT_SETATTR_FMT, (unsigned long)handle, (unsigned)strlen(text), text);
	Ble_Send_Command(resp);

	snprintf(resp, sizeof(resp), BLE_AT_NOTIFY_FMT, (unsigned long)conn_id, (unsigned long)handle,
			(unsigned)strlen(text), text);
	Ble_Send_Command(resp);
}

static void Ble_Update_WaitFlag(const char* line)
{
	if (line == NULL) {
		return;
	}
	if (strstr(line, "DISCONN") != NULL || strstr(line, "DISCONNECTED") != NULL) {
		ble_wait_for_ip = false;
		return;
	}
	if (strstr(line, "CONN") != NULL || strstr(line, "CONNECT") != NULL) {
		ble_wait_for_ip = true;
	}
}

static uint32_t Ble_Parse_First_Number(const char* line)
{
	const char* p = strchr(line, ':');
	if (p == NULL) {
		return 0;
	}
	p++;
	while (*p != '\0' && !isdigit((unsigned char)*p)) {
		p++;
	}
	if (!isdigit((unsigned char)*p)) {
		return 0;
	}
	return (uint32_t)strtoul(p, NULL, 10);
}

static uint32_t Ble_Parse_Last_Number(const char* line)
{
	size_t len = strlen(line);
	while (len > 0 && !isdigit((unsigned char)line[len - 1])) {
		len--;
	}
	if (len == 0) {
		return 0;
	}
	size_t start = len;
	while (start > 0 && isdigit((unsigned char)line[start - 1])) {
		start--;
	}
	return (uint32_t)strtoul(line + start, NULL, 10);
}
