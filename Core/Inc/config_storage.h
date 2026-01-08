/*
 * config_storage.h
 *
 *  Created on: Sep 18, 2025
 */

#ifndef INC_CONFIG_STORAGE_H_
#define INC_CONFIG_STORAGE_H_

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CONFIG_STORAGE_MAX_IP_LEN 16

bool ConfigStorage_LoadIp(char* out_ip, size_t out_len);
bool ConfigStorage_SaveIp(const char* ip);
bool ConfigStorage_IsValidIp(const char* ip);

#ifdef __cplusplus
}
#endif

#endif /* INC_CONFIG_STORAGE_H_ */
