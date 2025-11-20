/*
 * myclock.h
 *
 *  Created on: May 11, 2024
 *      Author: Blu
 */

#ifndef MYCLOCK_H_
#define MYCLOCK_H_

#include "stm32f10x.h"

void setto72MHz(void);


void setto72MHz(void)
{
	//turn on external oscillator
	RCC->CR |= RCC_CR_HSEON;
	//wait for HSE crystal to be stable
	while(!(RCC->CR & RCC_CR_HSERDY));
	//activate prefetch buffer
	FLASH->ACR |= FLASH_ACR_PRFTBE;
	//flash to wait state
	//reset just to be sure
	FLASH->ACR &= ~(FLASH_ACR_LATENCY);
	FLASH->ACR |= (uint32_t) 0x02;

	//configure RCC and PLL settings while PLL is off
	//reset
	RCC->CFGR &= ~((RCC_CFGR_PLLSRC) | (RCC_CFGR_PLLXTPRE) | (RCC_CFGR_PLLMULL));
	RCC->CFGR &= ~(RCC_CFGR_PLLXTPRE);

	RCC->CFGR |= RCC_CFGR_PLLSRC;
	RCC->CFGR |= RCC_CFGR_PLLMULL9;
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;

	//turn on PLL
	RCC->CR |= (RCC_CR_PLLON);
	while(!(RCC->CR & RCC_CR_PLLRDY));

	RCC->CFGR &= ~(RCC_CFGR_SW);
	RCC->CFGR |= RCC_CFGR_SW_PLL;
	while(!(RCC->CFGR & RCC_CFGR_SWS_PLL));

	SystemCoreClockUpdate();

	SysTick_Config(SystemCoreClock/1000);
}





#endif /* MYCLOCK_H_ */
