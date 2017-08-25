/*
 * Uart.h
 *
 *  Created on: 25 aug. 2017
 *      Author: JefPC
 */

#ifndef UART_H_
#define UART_H_

//
#include <stdint.h>

//
//
//
void Uart_Setup ();
int16_t Uart_Getc();

void Uart_Putc(char ch);
void Uart_Puts(const char *ch);

#endif /* UART_H_ */
