/*
 * mydebug.h
 *
 *  Created on: May 11, 2024
 *      Author: Blu
 */

#ifndef MYDEBUG_H_
#define MYDEBUG_H_

#include "stm32f10x.h"                  // Device header
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include "mygpio.h"

#define		DEBUG_UART USART1
#define delay		for(int i=0; i<5000000; i++)

void uart_init(void);
void uart_sendchar(char cdata);
char uart_getchar(void);
void uart_sendstring(char *message);
void printmsg(char *msg, ...);

/*
****************UART Debug Example*******************
//printmsg("Channel[5]: %d  |  Channel[7]: %d\n", samples[0], samples[1]);
void USART1_IRQHandler(void)
{
	if(USART1->SR & USART_SR_RXNE) //if RX is not empty
	{
		temp = USART1->DR;
		USART1->DR = temp;
		while(!(USART1->SR & USART_SR_TC));
	}

	if(temp == 'A') GPIO_Write(PORTC, 13, LOW);
	if((temp == 'B')) GPIO_Write(PORTC, 13, HIGH);
	temp = 0;

}
*/



void uart_init(void)
{
	RCC->APB2ENR |= PORTAEN | AFIOEN | UART1EN ;
	config_gpio_output(GPIOA, 9, out_50MHz, AlternateFunction);
	config_gpio_input(GPIOA, 10, PullUpPullDown);

	USART1->BRR = 0x1D4C;
	USART1 ->CR1 |= USART_CR1_TE;
	USART1 ->CR1 |= USART_CR1_UE;
}

void printmsg(char *msg, ...)
{
	char buff[80];
	#ifdef DEBUG_UART
		va_list args;
		va_start(args, msg);
		vsprintf(buff, msg, args);
		for(int i=0; i<strlen(buff); i++)
		{
			USART1->DR = buff[i];
			while( !(USART1->SR & USART_SR_TXE) );
		}
	#endif
}

void uart_sendchar(char cdata)
{
	USART1->DR = cdata;
	while( !(USART1->SR & USART_SR_TXE) );
}

void uart_sendstring(char *message)
{
	while(*message)
	{
		uart_sendchar(*message);
		message++;
	}
}

char uart_getchar(void)
{
	unsigned char tempdata;
	if(( !(USART1->SR & USART_SR_RXNE)))
	{
		tempdata =  USART1->DR;
	}
	return tempdata;
}




#endif /* MYDEBUG_H_ */
