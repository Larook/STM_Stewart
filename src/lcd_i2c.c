/*
 * lcd_i2c.c
 *
 *  Created on: 13.10.2019
 *      Author: PiK
 */

//TO DO:
//poprawic wysylanie i odbieranie
//zrobic uniwersalna biblioteke z wysylaniem I2C, zeby nie korzystac z nazw lsm/lcd
#include "stm32f10x.h"
#include "lcd_i2c.h"
#include <stdio.h>

void lcd_set_reg(uint8_t reg) {
	I2C_GenerateSTART(I2C1, ENABLE);
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS)
		;
	I2C_Send7bitAddress(I2C1, LCD_I2C_ADDRESS, I2C_Direction_Transmitter);
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)
			!= SUCCESS)
		;

	I2C_SendData(I2C1, reg); // dlaczego dodajemy 0x80?
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING) != SUCCESS)
		;
}

void lcd_write_reg(uint8_t reg, uint8_t value) {
	lcd_write(reg, &value, sizeof(value));
}
void lcd_write(uint8_t reg, const void* data, int size) {
	int i;
	const uint8_t* buffer = (uint8_t*) data;

	lcd_set_reg(reg);
	for (i = 0; i < size; i++) {
		I2C_SendData(I2C1, buffer[i]);
		while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING)
				!= SUCCESS)
			;
	}
	I2C_GenerateSTOP(I2C1, ENABLE);
}

uint8_t lcd_read_reg(uint8_t reg) {
	uint8_t value = 0;
	lcd_read(reg, &value, sizeof(value));
	return value;
}
// raczej nieuzywane
void lcd_read(uint8_t reg, void* data, int size) {
	int i;
	uint8_t* buffer = (uint8_t*) data;

	lcd_set_reg(reg); // !

	I2C_GenerateSTART(I2C1, ENABLE);
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS)
		;

	I2C_AcknowledgeConfig(I2C1, ENABLE);
	I2C_Send7bitAddress(I2C1, LCD_I2C_ADDRESS, I2C_Direction_Receiver);
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)
			!= SUCCESS)
		;

	for (i = 0; i < size - 1; i++) {
		while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) != SUCCESS)
			;
		buffer[i] = I2C_ReceiveData(I2C1);
	}
	I2C_AcknowledgeConfig(I2C1, DISABLE);
	I2C_GenerateSTOP(I2C1, ENABLE);
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) != SUCCESS)
		;
	buffer[i] = I2C_ReceiveData(I2C1);
}

//command function
void lcd_send_cmd(char cmd) {
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = cmd & 0xf0;    // select only upper nibble
	data_l = (cmd << 4) & 0xf0;    // select only lower nibble
	data_t[0] = data_u | 0x04;  //en=1, rs=0
	data_t[1] = data_u;  //en=0, rs=0
	data_t[2] = data_l | 0x04;  //en=1, rs=0
	data_t[3] = data_l;  //en=0, rs=0 LCD_I2C_ADDRESS
	lcd_write_reg(LCD_I2C_ADDRESS, cmd );
	//HAL_I2C_Master_Transmit(&hi2c1, 0x4E, (uint8_t *) data_t, 4, 100);
}

//data function
void lcd_send_data(char data) {
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = data & 0xf0;    // upper data nibble
	data_l = (data << 4) & 0xf0;    // lower data nibble
	data_t[0] = data_u | 0x05;  //en=1, rs=0
	data_t[1] = data_u | 0x01;  //en=0, rs=0
	data_t[2] = data_l | 0x05;  //en=1, rs=0
	data_t[3] = data_l | 0x01;  //en=0, rs=0
	lcd_write_reg(LCD_I2C_ADDRESS, data );
	//HAL_I2C_Master_Transmit(&hi2c1, 0x4E, (uint8_t *) data_t, 4, 100);
}

void lcd_init(void) {
	lcd_send_cmd(0x02);
	lcd_send_cmd(0x28);
	lcd_send_cmd(0x0c);
	lcd_send_cmd(0x80);
}

void lcd_send_string(char *str) {
	while (*str)
		lcd_send_data(*str++);
}
