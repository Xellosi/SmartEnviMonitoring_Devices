/*
 * config_storage.c
 *
 *  Created on: Sep 18, 2025
 */

#include "config_storage.h"

#include <string.h>

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx_hal_flash_ex.h"

#define CONFIG_STORAGE_MAGIC 0x434F4E46u
#define CONFIG_STORAGE_VERSION 1u

#define CONFIG_FLASH_SECTOR FLASH_SECTOR_11
#define CONFIG_FLASH_ADDR 0x080E0000u

typedef struct {
	uint32_t magic;
	uint16_t version;
	uint16_t length;
	char ip[CONFIG_STORAGE_MAX_IP_LEN];
	uint32_t crc;
} ConfigStorageRecord;

static uint32_t ConfigStorage_Crc32(const uint8_t* data, size_t len);
static bool ConfigStorage_ReadRecord(ConfigStorageRecord* record);
static bool ConfigStorage_WriteRecord(const ConfigStorageRecord* record);

bool ConfigStorage_IsValidIp(const char* ip)
{
	if (ip == NULL) {
		return false;
	}

	int octet = 0;
	int octet_count = 0;
	int digit_count = 0;
	const char* p = ip;
	while (*p != '\0') {
		if (*p >= '0' && *p <= '9') {
			octet = octet * 10 + (*p - '0');
			if (octet > 255) {
				return false;
			}
			digit_count++;
			if (digit_count > 3) {
				return false;
			}
		} else if (*p == '.') {
			if (digit_count == 0) {
				return false;
			}
			octet_count++;
			octet = 0;
			digit_count = 0;
		} else {
			return false;
		}
		p++;
	}

	if (digit_count == 0) {
		return false;
	}
	return octet_count == 3;
}

bool ConfigStorage_LoadIp(char* out_ip, size_t out_len)
{
	if (out_ip == NULL || out_len == 0) {
		return false;
	}

	ConfigStorageRecord record;
	if (!ConfigStorage_ReadRecord(&record)) {
		return false;
	}

	if (record.length == 0 || record.length >= CONFIG_STORAGE_MAX_IP_LEN) {
		return false;
	}

	if (!ConfigStorage_IsValidIp(record.ip)) {
		return false;
	}

	size_t copy_len = record.length;
	if (copy_len >= out_len) {
		copy_len = out_len - 1;
	}
	memcpy(out_ip, record.ip, copy_len);
	out_ip[copy_len] = '\0';
	return true;
}

bool ConfigStorage_SaveIp(const char* ip)
{
	if (!ConfigStorage_IsValidIp(ip)) {
		return false;
	}

	ConfigStorageRecord current;
	if (ConfigStorage_ReadRecord(&current)) {
		if (strncmp(current.ip, ip, CONFIG_STORAGE_MAX_IP_LEN) == 0) {
			return true;
		}
	}

	ConfigStorageRecord record;
	memset(&record, 0, sizeof(record));
	record.magic = CONFIG_STORAGE_MAGIC;
	record.version = CONFIG_STORAGE_VERSION;
	record.length = (uint16_t)strlen(ip);
	strncpy(record.ip, ip, sizeof(record.ip) - 1);
	record.crc = ConfigStorage_Crc32((const uint8_t*)&record, sizeof(record) - sizeof(record.crc));

	return ConfigStorage_WriteRecord(&record);
}

static bool ConfigStorage_ReadRecord(ConfigStorageRecord* record)
{
	if (record == NULL) {
		return false;
	}

	const ConfigStorageRecord* stored = (const ConfigStorageRecord*)CONFIG_FLASH_ADDR;
	if (stored->magic != CONFIG_STORAGE_MAGIC) {
		return false;
	}
	if (stored->version != CONFIG_STORAGE_VERSION) {
		return false;
	}

	memcpy(record, stored, sizeof(*record));
	uint32_t crc = ConfigStorage_Crc32((const uint8_t*)record, sizeof(*record) - sizeof(record->crc));
	if (crc != record->crc) {
		return false;
	}
	return true;
}

static bool ConfigStorage_WriteRecord(const ConfigStorageRecord* record)
{
	if (record == NULL) {
		return false;
	}

	HAL_FLASH_Unlock();

	FLASH_EraseInitTypeDef erase;
	uint32_t sector_error = 0;
	memset(&erase, 0, sizeof(erase));
	erase.TypeErase = FLASH_TYPEERASE_SECTORS;
	erase.Sector = CONFIG_FLASH_SECTOR;
	erase.NbSectors = 1;
	erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

	if (HAL_FLASHEx_Erase(&erase, &sector_error) != HAL_OK) {
		HAL_FLASH_Lock();
		return false;
	}

	const uint8_t* data = (const uint8_t*)record;
	uint32_t address = CONFIG_FLASH_ADDR;
	for (size_t i = 0; i < sizeof(*record); i += sizeof(uint32_t)) {
		uint32_t word = 0xFFFFFFFFu;
		size_t remaining = sizeof(*record) - i;
		memcpy(&word, data + i, remaining >= sizeof(uint32_t) ? sizeof(uint32_t) : remaining);
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, word) != HAL_OK) {
			HAL_FLASH_Lock();
			return false;
		}
		address += sizeof(uint32_t);
	}

	HAL_FLASH_Lock();
	return true;
}

static uint32_t ConfigStorage_Crc32(const uint8_t* data, size_t len)
{
	uint32_t crc = 0xFFFFFFFFu;
	for (size_t i = 0; i < len; ++i) {
		crc ^= data[i];
		for (uint32_t bit = 0; bit < 8; ++bit) {
			if (crc & 1u) {
				crc = (crc >> 1) ^ 0xEDB88320u;
			} else {
				crc >>= 1;
			}
		}
	}
	return ~crc;
}
