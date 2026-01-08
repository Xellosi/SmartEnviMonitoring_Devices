/*
 * ble_config.h
 *
 *  Created on: Sep 18, 2025
 */

#ifndef INC_BLE_CONFIG_H_
#define INC_BLE_CONFIG_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BLE_CONFIG_SERVICE_UUID "12345678-1234-5678-1234-56789ABCDEF0"
#define BLE_CONFIG_CHAR_UUID "12345678-1234-5678-1234-56789ABCDEF1"

typedef void (*BleConfigApplyFn)(const char* ip, bool persist, bool reconnect);

void Ble_Config_SetApplyFn(BleConfigApplyFn fn);
void Ble_Config_Init(void);
void Ble_Config_HandleRx(const uint8_t* data, size_t len);
bool Ble_Config_ShouldWaitIp(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_BLE_CONFIG_H_ */
