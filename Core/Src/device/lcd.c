/*
 * lcd1602a.c
 *
 *  Created on: Jul 12, 2024
 *      Author: hung
 */

//LCD: LCD1602
//i2C Module: PCF8574

#include <stdint.h>
#include <stdbool.h>

#include "string.h"
#include "stm32f4xx.h"
#include "utils.h"
#include "lcd.h"


uint32_t tranTimeout = 1000;

HAL_StatusTypeDef lcd_send_cmd(I2C_HandleTypeDef *i2chandle, char cmd) {
	char data_h, data_l;
	uint8_t frame_data[4];
	data_h = (cmd & 0xf0);
	data_l = ((cmd << 4) & 0xf0);
	frame_data[0] = data_h | 0x0C;    //en=1, rs=0
	frame_data[1] = data_h | 0x08;    //en=0, rs=0
	frame_data[2] = data_l | 0x0C;    //en=1, rs=0
	frame_data[3] = data_l | 0x08;    //en=0, rs=0

	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(i2chandle, LCD_ADDR,
			(uint8_t*) frame_data, 4, tranTimeout);

	HAL_Delay(1);

	return status;
}

HAL_StatusTypeDef lcd_send_data(I2C_HandleTypeDef *i2chandle, char data) {
	char data_h, data_l;
	uint8_t frame_data[4];
	data_h = (data & 0xf0);
	data_l = ((data << 4) & 0xf0);
	frame_data[0] = data_h | 0x0D;    //en=1, rs=1
	frame_data[1] = data_h | 0x09;    //en=0, rs=1
	frame_data[2] = data_l | 0x0D;    //en=1, rs=1
	frame_data[3] = data_l | 0x09;    //en=0, rs=1

	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(i2chandle, LCD_ADDR,
			(uint8_t*) frame_data, 4, tranTimeout);
	HAL_Delay(1);

	return status;
}

void LCD_Clear(I2C_HandleTypeDef *i2chandle) {
	lcd_send_cmd(i2chandle, 0x01);
	HAL_Delay(1);
}

void LCD_Init(I2C_HandleTypeDef *i2chandle) {
	HAL_StatusTypeDef status = HAL_OK;
	HAL_Delay(200);
	status = lcd_send_cmd(i2chandle, 0x30);
	HAL_Delay(5);
	status = lcd_send_cmd(i2chandle, 0x30);
	HAL_Delay(1);
	status = lcd_send_cmd(i2chandle, 0x30);
	HAL_Delay(10);
	status = lcd_send_cmd(i2chandle, 0x20);
	HAL_Delay(10);

	status = lcd_send_cmd(i2chandle, 0x28);        //function set
	HAL_Delay(1);
	status = lcd_send_cmd(i2chandle, 0x08);        //Display on/off
	HAL_Delay(1);
	status = lcd_send_cmd(i2chandle, 0x01);        //clear display
	HAL_Delay(1);
	status = lcd_send_cmd(i2chandle, 0x06);        //Enter mode
	HAL_Delay(1);
	status = lcd_send_cmd(i2chandle, 0x0C);        //Display on/off
	HAL_Delay(1);
}

void LCD_Send_String(I2C_HandleTypeDef *i2chandle, char *str) {
	char* tmp = str;
	while (*tmp >= 32) {
		lcd_send_data(i2chandle, *tmp);
		//printf("%c",*tmp);
		HAL_Delay(10);
		tmp++;
	}
	HAL_Delay(1);
}

void LCD_Put_Cur(I2C_HandleTypeDef *i2chandle, uint8_t row, uint8_t col) {
	lcd_send_cmd(i2chandle, 0x80 | (col + (0x40 * row)));
}

ErrorStatus LCD_Refresh(I2C_HandleTypeDef *i2chandle, LCDContent_t* content){
	if (content == NULL){
		return ERROR;
	}
	content -> FirstLine[LCD_CHAR_NUM] = '\0';
	content -> SecLine[LCD_CHAR_NUM] = '\0';
	HAL_Delay(10);
	LCD_Clear(i2chandle);
	LCD_Put_Cur(i2chandle, 0, 0);
	HAL_Delay(10);
	LCD_Send_String(i2chandle, content -> FirstLine);
	HAL_Delay(10);
	LCD_Put_Cur(i2chandle, 1, 0);
	HAL_Delay(10);
	LCD_Send_String(i2chandle, content -> SecLine);

	printf("%s\t%s\n",content -> FirstLine, content -> SecLine);
	return SUCCESS;
}

ErrorStatus LCD_SetContent_Refresh(I2C_HandleTypeDef *i2chandle, LCDContent_t* content, char* line1, char* line2){
	if (content == NULL){
		return ERROR;
	}

	if(line1 != NULL){
		int len1 = fmin(LCD_CHAR_NUM, strlen(line1));
		memcpy(content -> FirstLine, line1, len1);
		content -> FirstLine[len1] = '\0';
	}
	if(line2 != NULL){
		int len2 = fmin(LCD_CHAR_NUM, strlen(line2));
		memcpy(content -> SecLine, line2, len2);
		content -> SecLine[len2] = '\0';
	}
	return LCD_Refresh(i2chandle, content);
}

void _test(I2C_HandleTypeDef *i2chandle) {
	LCD_Put_Cur(i2chandle, 0, 0);
	LCD_Send_String(i2chandle, "Hello");
	LCD_Put_Cur(i2chandle, 1, 0);
	LCD_Send_String(i2chandle, "World");
	HAL_Delay(1000);
	LCD_Clear(i2chandle);
	HAL_Delay(1000);
}
