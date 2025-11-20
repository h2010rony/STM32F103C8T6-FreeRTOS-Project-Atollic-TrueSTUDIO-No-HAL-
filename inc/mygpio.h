/*
 * mygpio.h
 *
 *  Created on: May 11, 2024
 *      Author: Blu
 */

#ifndef MYGPIO_H_
#define MYGPIO_H_

#include <stdint.h>
#include <stddef.h>
#include "stm32f10x.h"

#define LOW		0
#define HIGH 	1


/*GPIO OUTPUT definition*/
#define PORTA	GPIOA->ODR
#define PORTB	GPIOB->ODR
#define PORTC	GPIOC->ODR
#define PORTD	GPIOD->ODR
#define PORTE	GPIOE->ODR
#define PORTF	GPIOF->ODR
#define PORTG	GPIOG->ODR



/*GPIO Input register*/
#define PINA	GPIOA->IDR
#define PINB	GPIOB->IDR
#define PINC	GPIOC->IDR
#define PIND	GPIOD->IDR
#define PINE	GPIOE->IDR
#define PINF	GPIOF->IDR
#define PING	GPIOG->IDR

/*Pin Mode*/
#define OUTPUT			(uint32_t) (0x01)
#define INPUT			(uint32_t) (0x02)

/*Output Modes*/
#define GeneralPurpose					(uint32_t) (0x00)
#define	OpenDrain						(uint32_t) (0x01)
#define	AlternateFunction				(uint32_t) (0x02)
#define	AlternateFunction_OpenDrain		(uint32_t) (0x03)

/*Input Modes*/
#define Analog							(uint32_t) (0x00)
#define	Floating						(uint32_t) (0x01)
#define	PullUpPullDown					(uint32_t) (0x02)

/*Pin speed*/
#define out_2MHz	(uint32_t) (0x02)
#define out_10MHz	(uint32_t) (0x01)
#define out_50MHz	(uint32_t) (0x03)

/*Clock enable*/




/*Enable peripheral clock AHBENR*/
#define enable_dma1_clock		RCC->AHBENR |= (1 << 0)
#define enable_dma2_clock		RCC->AHBENR |= (1 << 1)
#define enable_sram_clock		RCC->AHBENR |= (1 << 2)
#define enable_flitf_clock		RCC->AHBENR |= (1 << 4)
#define enable_crc_clock		RCC->AHBENR |= (1 << 6)
#define enable_fsmc_clock		RCC->AHBENR |= (1 << 8)
#define enable_sdio_clock		RCC->AHBENR |= (1 << 10)


/*Enable peripheral clock APB2ENR*/
/*Enable peripheral clock APB2ENR*/
#define AFIOEN		(1 << 0)
#define PORTAEN		(1 << 2)
#define PORTBEN		(1 << 3)
#define PORTCEN		(1 << 4)
#define PORTDEN		(1 << 5)
#define PORTEEN		(1 << 6)
#define PORTFEN		(1 << 7)
#define PORTGEN		(1 << 8)
#define ADC1EN		(1 << 9)
#define ADC2EN		(1 << 10)
#define TIM1EN		(1 << 11)
#define SPI1EN		(1 << 12)
#define TIM8EN		(1 << 13)
#define UART1EN		(1 << 14)
#define ADC3EN		(1 << 15)
#define TIM9EN		(1 << 19)
#define TIM10EN		(1 << 20)
#define TIM11EN		(1 << 21)

#define enable_afio_clock		RCC->APB2ENR |= (1 << 0)
#define enable_porta_clock		RCC->APB2ENR |= (1 << 2)
#define enable_portb_clock		RCC->APB2ENR |= (1 << 3)
#define enable_portc_clock		RCC->APB2ENR |= (1 << 4)
#define enable_portd_clock		RCC->APB2ENR |= (1 << 5)
#define enable_porte_clock		RCC->APB2ENR |= (1 << 6)
#define enable_portf_clock		RCC->APB2ENR |= (1 << 7)
#define enable_portg_clock		RCC->APB2ENR |= (1 << 8)
#define enable_adc1_clock		RCC->APB2ENR |= (1 << 9)
#define enable_adc2_clock		RCC->APB2ENR |= (1 << 10)
#define enable_tim1_clock		RCC->APB2ENR |= (1 << 11)
#define enable_spi1_clock		RCC->APB2ENR |= (1 << 12)
#define enable_tim8_clock		RCC->APB2ENR |= (1 << 13)
#define enable_uart1_clock		RCC->APB2ENR |= (1 << 14)
#define enable_adc3_clock		RCC->APB2ENR |= (1 << 15)
#define enable_tim9_clock		RCC->APB2ENR |= (1 << 19)
#define enable_tim10_clock		RCC->APB2ENR |= (1 << 20)
#define enable_tim11_clock		RCC->APB2ENR |= (1 << 21)


/*Enable peripheral clock APB1ENR*/
#define enable_tim2_clock		RCC->APB1ENR |= (1 << 0)
#define enable_tim3_clock		RCC->APB1ENR |= (1 << 1)
#define enable_tim4_clock		RCC->APB1ENR |= (1 << 2)
#define enable_tim5_clock		RCC->APB1ENR |= (1 << 3)
#define enable_tim6_clock		RCC->APB1ENR |= (1 << 4)
#define enable_tim7_clock		RCC->APB1ENR |= (1 << 5)
#define enable_tim12_clock		RCC->APB1ENR |= (1 << 6)
#define enable_tim13_clock		RCC->APB1ENR |= (1 << 7)
#define enable_tim14_clock		RCC->APB1ENR |= (1 << 8)
#define enable_wwdg_clock		RCC->APB1ENR |= (1 << 11)
#define enable_spi2_clock		RCC->APB1ENR |= (1 << 14)
#define enable_spi3_clock		RCC->APB1ENR |= (1 << 15)
#define enable_uart2_clock		RCC->APB1ENR |= (1 << 17)
#define enable_uart3_clock		RCC->APB1ENR |= (1 << 18)
#define enable_uart4_clock		RCC->APB1ENR |= (1 << 19)
#define enable_uart5_clock		RCC->APB1ENR |= (1 << 20)
#define enable_i2c1_clock		RCC->APB1ENR |= (1 << 21)
#define enable_i2c2_clock		RCC->APB1ENR |= (1 << 22)
#define enable_usb_clock		RCC->APB1ENR |= (1 << 23)
#define enable_can_clock		RCC->APB1ENR |= (1 << 25)
#define enable_bkp_clock		RCC->APB1ENR |= (1 << 27)
#define enable_pwr_clock		RCC->APB1ENR |= (1 << 28)
#define enable_dac_clock		RCC->APB1ENR |= (1 << 29)


/* Clock control register RCC->CR*/
#define enable_hsi		RCC->CR |= (1 << 0)
#define enable_hse		RCC->CR |= (1 << 16)


#define CNF_BIT1 	(pinpos[PinNumber]+2)
#define CNF_BIT2 	(pinpos[PinNumber]+3)




typedef enum
{
	RisingEdge,
	FallingEdge,
	RisingFallingEdge
}edge_select;


#define pin_is_set(gpionumbrt, pingpio) gpionumbrt->IDR & (1 << pingpio)



void config_gpio_input(GPIO_TypeDef *port, uint32_t PinNumber, uint32_t mode_type);
void config_gpio_output(GPIO_TypeDef *port, uint32_t PinNumber, uint32_t speed, uint32_t mode_type);
void config_gpio_interrupt(GPIO_TypeDef *port, uint32_t PinNumber, edge_select edge);
void enable_gpio_interrupt(uint32_t PinNumber, IRQn_Type irqNumber);
void gpio_write(GPIO_TypeDef *port, uint32_t pinnumber, uint32_t PinState);
void gpio_set(GPIO_TypeDef *port, uint32_t pinnumber);
void gpio_clear(GPIO_TypeDef *port, uint32_t pinnumber);
void gpio_toggle(GPIO_TypeDef *port, uint32_t pinnumber);
uint8_t gpio_read(GPIO_TypeDef *port, uint32_t pinnumber);

/*
	*********GPIO Configuration Example*********
	config_gpio_input(DDRB, 4, PullUpPullDown);
	config_gpio_output(DDRC, 13, out_2MHz, GeneralPurpose);
*/


/*
	************interrupt Example***************
	config_gpio_interrupt(DDRB, 4, FallingEdge);
	enable_gpio_interrupt(4, EXTI4_IRQn);

	Interupt Handler
	void EXTI4_IRQHandler(void)

*/





#endif  /* MYGPIO_H_ */
