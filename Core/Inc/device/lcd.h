/*
 * lcd1602a.h
 *
 *  Created on: Jul 12, 2024
 *      Author: hung
 */

#ifndef INC_DEVICE_LCD_H_
#define INC_DEVICE_LCD_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx.h"

#define LCD_CHAR_NUM (16)
#define LCD_LINE_LEN (LCD_CHAR_NUM + 1)

typedef struct {
	char FirstLine[LCD_LINE_LEN];
	char SecLine[LCD_LINE_LEN];

}LCDContent_t;

#define LCD_ADDR (0x27 << 1)

void LCD_Init(I2C_HandleTypeDef *i2chandle);
void LCD_Clear(I2C_HandleTypeDef *i2chandle);
void LCD_Send_String(I2C_HandleTypeDef *i2chandle, char *str);
void LCD_Put_Cur(I2C_HandleTypeDef *i2chandle, uint8_t row, uint8_t col);
ErrorStatus LCD_Refresh(I2C_HandleTypeDef *i2chandle, LCDContent_t* content);
ErrorStatus LCD_SetContent_Refresh(I2C_HandleTypeDef *i2chandle, LCDContent_t* content, char* line1, char* line2);
#endif /* INC_DEVICE_LCD_H_ */
