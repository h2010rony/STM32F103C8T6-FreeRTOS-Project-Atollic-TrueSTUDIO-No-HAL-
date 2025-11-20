/*
 * mydelay.h
 *
 *  Created on: May 11, 2024
 *      Author: Blu
 */

#ifndef MYDELAY_H_
#define MYDELAY_H_


#include "stm32f10x.h"

void delayp(void);
void delayu(void);
void systick_init(void);
void delay_ms(unsigned long t);
void delay_us(unsigned long t);
void sysTick_Handler(void);
void Delay (uint32_t dlyTicks);


void systick_init(void)
{
	SysTick->CTRL = 0;
	SysTick->LOAD = 0x00FFFFFF;
	SysTick->VAL = 0;
	SysTick->CTRL |= 5;
}

void delayp(void)
{
	SysTick->LOAD = 0x11940;
	SysTick->VAL = 0;
	while((SysTick->CTRL & 0x00010000) == 0);
}


void delayu(void)
{
	SysTick->LOAD = 0x48;
	SysTick->VAL = 0;
	while((SysTick->CTRL & 0x00010000) == 0);
}

void delay_ms(unsigned long t)
{
	for(;t>0;t--)
		{
			delayp();
		}
}

void delay_us(unsigned long t)
{
	for(;t>0;t--)
		{
			delayu();
		}
}

static volatile uint32_t msTicks;
    //! The interrupt handler for the SysTick module

void sysTick_Handler(void)
{
	msTicks++;
}

void Delay (uint32_t dlyTicks)
{
	uint32_t curTicks;
	curTicks = msTicks;
	while ((msTicks - curTicks) < dlyTicks) { __NOP(); }
}






#endif /* MYDELAY_H_ */
