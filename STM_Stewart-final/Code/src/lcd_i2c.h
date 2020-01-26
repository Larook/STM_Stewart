/*
 * lcd_i2c.h
 *
 *  Created on: 13.10.2019
 *      Author: PiK
 */

#ifndef LCD_I2C_H_
#define LCD_I2C_H_

// jesli PCF8574A to salve address = 01110000
#define LCD_I2C_ADDRESS			0x4E // bit 0=R/W , bit bit 8=start condition

// PCF854 wartosci waznych bitów
#define E 		1<<4
#define RS		1<<5
#define RW		1<<6

// LCD COMMANDS
#define CLEAR_DISPLAY 				0x01
#define RETURN_HOME 				0x02
#define DECREMENT_CURSOR 			0x04
#define INCREMENT_CURSOR 			0x06
#define SHIFT_DISP_RIGHT 			0x05
#define SHIFT_DISP_LEFT 			0x07
#define DISP_OFF_CURSOR_OFF 		0x08
#define DISP_OFF_CURSOR_ON 			0x0A
#define DISP_ON_CURSOR_OFF 			0x0C
#define DISP_OF_CURSOR_BLINK		0x0E
#define DISP_ON_CURSOR_BLINK 		0x0F
#define SHIFT_CURSOR_LEFT			0x10
#define SHIFT_CURSOR_RIGHT			0x14
#define SHIFT_DISPLAY_LEFT 			0x18
#define SHIFT_DISPLAY_RIGHT 		0x1C
#define CURSOR_TO_FIRST				0x80
#define CURSOR_TO_SECOND 			0xC0
#define TWO_LINES 					0x38







void lcd_send_4bit(uint8_t data);
void lcd_send_two_4bit(uint8_t data1,uint8_t data2);

void lcd_write_reg(uint8_t reg, uint8_t value);
void lcd_write(uint8_t reg, const void* data, int size) ;

uint8_t lcd_read_reg(uint8_t reg);

// raczej nieuzywane
void lsm_read(uint8_t reg, void* data, int size);
//command function
void lcd_send_cmd(char cmd);
//data function
void lcd_send_data(char data);
void lcd_init(void);
void lcd_send_string(char *str);


#endif /* LCD_I2C_H_ */
