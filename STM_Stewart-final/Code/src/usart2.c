/*
 * usart2.c
 *
 *  Created on: 08.02.2019
 *      Author: PiK
 */
#include "stm32f10x.h"
#include "usart2.h"

void send_char(char c) {
	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
		;
	USART_SendData(USART2, c);
}

int __io_putchar(int c) {
	if (c == '\n')
		send_char('\r');
	send_char(c);
	return c;
}
