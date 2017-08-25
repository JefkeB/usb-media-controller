/*
 * Uart.c
 *
 *  Created on: 25 aug. 2017
 *      Author: JefPC
 */

//
//
//
#include "Uart.h"

#include <stm32f103xb.h>


//
//
//
void Uart_Setup ()
{
  // make sure the relevant pins are appropriately set up.
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;              // enable clock for GPIOA
  GPIOA->CRH   |= (0x0BUL  << 4);                  // Tx (PA9) alt. out push-pull
  GPIOA->CRH   |= (0x04UL  << 8);                  // Rx (PA10) in floating
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;            // enable clock for USART1
  USART1->BRR  = 72000000L/115200L;                // set baudrate
  USART1->CR1 |= (USART_CR1_RE | USART_CR1_TE);  // RX, TX enable
  USART1->CR1 |= USART_CR1_UE;                    // USART enable
}


//
//
//
int16_t Uart_Getc()
{
	//while (!(USART1->SR & USART_SR_RXNE));
	if(USART1->SR & USART_SR_RXNE)
	{
		return ((int16_t)(USART1->DR & 0xFF));
	}

	return -1;
}


//
//
//
void Uart_Putc(char ch)
{
	while (!(USART1->SR & USART_SR_TXE));

	USART1->DR = ch;
}


//
//
//
void Uart_Puts(char *s)
{
	while(*s)
    {
        Uart_Putc(*s++);
    }
}
