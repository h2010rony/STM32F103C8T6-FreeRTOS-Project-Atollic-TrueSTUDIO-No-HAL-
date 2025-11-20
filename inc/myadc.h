/*
 * myadc.h
 *
 *  Created on: May 11, 2024
 *      Author: Blu
 */

#ifndef MYADC_H_
#define MYADC_H_

#include <stdint.h>
#include "stm32f10x.h"
#include "mydelay.h"
#include "mygpio.h"
#include "mydma.h"


#define adc_conversion_finished		ADC1->SR & (1 << 1)

/*ADC Status Refister ADC->SR*/
#define endofconversion	ADC->SR & (1 << 1)


/*ADC Control Register ADC->CR1*/
#define enable_eoc_interrupt		ADC1->CR1 |= (1 << 5)
#define enable_awdg_interrupt		ADC1->CR1 |= (1 << 6)
#define enable_scan					ADC1->CR1 |= (1 << 8)
#define enable_awdsgl				ADC1->CR1 |= (1 << 9)
#define enable_disc					ADC1->CR1 |= (1 << 11)


/*ADC Control Register ADC->CR2*/
#define start_conversion			ADC1->CR2 |= (1 << 22)
#define adc_on						ADC1->CR2 |= (1 << 0)
#define adc_calibration				ADC1->CR2 |= (1 << 2)
#define adc_calibration_reset		ADC1->CR2 |= (1 << 3)
#define adc_dma_mode				ADC1->CR2 |= (1 << 8)


#define ADON	(1 << 0)
#define CONT	(1 << 1)
#define adc_on_continuous_mode	ADC1->CR2 |= ADON | CONT

/*ADC Prescaler*/
#define adc_precaler_div4		RCC->CFGR |= (1 << 14)
#define adc_precaler_div6		RCC->CFGR |= (1 << 15)
#define adc_precaler_div8		RCC->CFGR |= (1 << 14) | (1 << 15)


/*ADC Sample time register ADC->SMPR1 and ADC->SMPR2*/
#define adc_channel_0(sampletime)	ADC1->SMPR2 |= (sampletime<<0)
#define adc_channel_1(sampletime)	ADC1->SMPR2 |= (sampletime<<3)
#define adc_channel_2(sampletime)	ADC1->SMPR2 |= (sampletime<<6)
#define adc_channel_3(sampletime)	ADC1->SMPR2 |= (sampletime<<9)
#define adc_channel_4(sampletime)	ADC1->SMPR2 |= (sampletime<<12)
#define adc_channel_5(sampletime)	ADC1->SMPR2 |= (sampletime<<15)
#define adc_channel_6(sampletime)	ADC1->SMPR2 |= (sampletime<<18)
#define adc_channel_7(sampletime)	ADC1->SMPR2 |= (sampletime<<21)
#define adc_channel_8(sampletime)	ADC1->SMPR2 |= (sampletime<<24)
#define adc_channel_9(sampletime)	ADC1->SMPR2 |= (sampletime<<27)
#define adc_channel_10(sampletime)	ADC1->SMPR1 |= (sampletime<<0)
#define adc_channel_11(sampletime)	ADC1->SMPR1 |= (sampletime<<3)
#define adc_channel_12(sampletime)	ADC1->SMPR1 |= (sampletime<<6)
#define adc_channel_13(sampletime)	ADC1->SMPR1 |= (sampletime<<9)
#define adc_channel_14(sampletime)	ADC1->SMPR1 |= (sampletime<<12)
#define adc_channel_15(sampletime)	ADC1->SMPR1 |= (sampletime<<15)
#define adc_channel_16(sampletime)	ADC1->SMPR1 |= (sampletime<<18)
#define adc_channel_17(sampletime)	ADC1->SMPR1 |= (sampletime<<21)


/*Sequence settings for ADC ADC1->SQRx*/
#define number_of_conversion(convnumber)	ADC1->SQR1 |= ((convnumber-1) << 20)

#define sequence_1(channelnumber)		ADC1->SQR3 |= (channelnumber << 0)
#define sequence_2(channelnumber)		ADC1->SQR3 |= (channelnumber << 5)
#define sequence_3(channelnumber)		ADC1->SQR3 |= (channelnumber << 10)
#define sequence_4(channelnumber)		ADC1->SQR3 |= (channelnumber << 15)
#define sequence_5(channelnumber)		ADC1->SQR3 |= (channelnumber << 20)
#define sequence_6(channelnumber)		ADC1->SQR3 |= (channelnumber << 25)

#define sequence_7(channelnumber)		ADC1->SQR2 |= (channelnumber << 0)
#define sequence_8(channelnumber)		ADC1->SQR2 |= (channelnumber << 5)
#define sequence_9(channelnumber)		ADC1->SQR2 |= (channelnumber << 10)
#define sequence_10(channelnumber)		ADC1->SQR2 |= (channelnumber << 15)
#define sequence_11(channelnumber)		ADC1->SQR2 |= (channelnumber << 20)
#define sequence_12(channelnumber)		ADC1->SQR2 |= (channelnumber << 25)

#define sequence_13(channelnumber)		ADC1->SQR1 |= (channelnumber << 0)
#define sequence_14(channelnumber)		ADC1->SQR1 |= (channelnumber << 5)
#define sequence_15(channelnumber)		ADC1->SQR1 |= (channelnumber << 10)
#define sequence_16(channelnumber)		ADC1->SQR1 |= (channelnumber << 15)



void adc_sequence(uint32_t sequence, uint32_t channel);
void adc_sampletime(uint32_t channel, uint32_t sampletimevalue);
void adc_cont_init(GPIO_TypeDef *port, uint32_t adcchannel, uint32_t sequence, uint32_t sampletime); //read data from ADC->DR
void adc_init_interrupt(GPIO_TypeDef *port, uint32_t adcchannel, uint32_t sequence, uint32_t sampletime);
void adc_init_dmamode(volatile uint16_t *adcvalues);



void adc_sequence(uint32_t sequence, uint32_t channel)
{

		switch(sequence)
		{
			case 1: ADC1->SQR3 |= (channel << 0);
					break;

			case 2: ADC1->SQR3 |= (channel << 5);
					break;

			case 3: ADC1->SQR3 |= (channel << 10);
					break;

			case 4: ADC1->SQR3 |= (channel << 15);
					break;

			case 5: ADC1->SQR3 |= (channel << 20);
					break;

			case 6: ADC1->SQR3 |= (channel << 25);
					break;

			case 7: ADC1->SQR2 |= (channel << 0);
					break;

			case 8: ADC1->SQR2 |= (channel << 5);
					break;

			case 9: ADC1->SQR2 |= (channel << 10);
					break;

			case 10: ADC1->SQR2 |= (channel << 15);
					 break;

			case 11: ADC1->SQR2 |= (channel << 20);
					 break;

			case 12: ADC1->SQR2 |= (channel << 25);
					 break;

			case 13: ADC1->SQR1 |= (channel << 0);
					 break;

			case 14: ADC1->SQR1 |= (channel << 5);
					 break;

			case 15: ADC1->SQR1 |= (channel << 10);
					 break;

			case 16: ADC1->SQR1 |= (channel << 15);
					 break;

		}
}


void adc_sampletime(uint32_t channel, uint32_t sampletimevalue)
{
	switch(channel-1)
	{
		case 0	:	ADC1->SMPR2 |= (sampletimevalue << 0);
					break;

		case 1	:	ADC1->SMPR2 |= (sampletimevalue << 3);
					break;

		case 2	:	ADC1->SMPR2 |= (sampletimevalue << 6);
					break;

		case 3	:	ADC1->SMPR2 |= (sampletimevalue << 9);
					break;

		case 4	:	ADC1->SMPR2 |= (sampletimevalue << 12);
					break;

		case 5	:	ADC1->SMPR2 |= (sampletimevalue << 15);
					break;

		case 6	:	ADC1->SMPR2 |= (sampletimevalue << 18);
					break;

		case 7	:	ADC1->SMPR2 |= (sampletimevalue << 21);
					break;

		case 8	:	ADC1->SMPR2 |= (sampletimevalue << 24);
					break;

		case 9	:	ADC1->SMPR2 |= (sampletimevalue << 27);
					break;

		case 10	:	ADC1->SMPR1 |= (sampletimevalue << 0);
					break;

		case 11	:	ADC1->SMPR1 |= (sampletimevalue << 3);
					break;

		case 12	:	ADC1->SMPR1 |= (sampletimevalue << 6);
					break;

		case 13	:	ADC1->SMPR1 |= (sampletimevalue << 9);
					break;

		case 14	:	ADC1->SMPR1 |= (sampletimevalue << 12);
					break;

		case 15	:	ADC1->SMPR1 |= (sampletimevalue << 15);
					break;

		case 16	:	ADC1->SMPR1 |= (sampletimevalue << 18);
					break;

		case 17	:	ADC1->SMPR1 |= (sampletimevalue << 21);
					break;

	}
}




void adc_cont_init(GPIO_TypeDef *port, uint32_t adcchannel, uint32_t sequence, uint32_t sampletime)
{
	adc_precaler_div6;
	config_gpio_input(port, adcchannel, Analog); //set channel 5 as analog input
	adc_sampletime(adcchannel, sampletime); 			// adc channel , 28.5 cycle = 2.375 us
	adc_sequence(sequence, adcchannel); 				// sequence , ad c channel
	enable_scan;						//enable scan
	adc_on_continuous_mode;	//enable ADC (continuous mode) for the first time
	delay_ms(1);

	adc_on;									//enable ADC  for the second time
	delay_ms(1);
	adc_calibration;
	while(ADC1->CR2 & ADC_CR2_CAL);
	delay_ms(2);
}

void adc_init_interrupt(GPIO_TypeDef *port, uint32_t adcchannel, uint32_t sequence, uint32_t sampletime)
{
	adc_precaler_div6;
	config_gpio_input(port, 5, Analog); //set channel 5 as analog input

	adc_sampletime(adcchannel, sampletime); 			// adc channel , 28.5 cycle = 2.375 us
	adc_sequence(sequence, adcchannel); 				// sequence , ad c channel

	enable_eoc_interrupt;
	NVIC_EnableIRQ(ADC1_2_IRQn);

	adc_on_continuous_mode;	//enable ADC (continuous mode) for the first time
	delay_ms(1);

	adc_on;									//enable ADC  for the second time
	delay_ms(1);
	adc_calibration;
	while(ADC1->CR2 & ADC_CR2_CAL);
	delay_ms(2);
}


void adc_init_dmamode(volatile uint16_t *adcvalues)
{
	adc_precaler_div6; ////change ADC prescaler not to exceed 14 MHz
	enable_dma1_clock;
	config_gpio_input(GPIOA, 5, Analog);
	config_gpio_input(GPIOA, 7, Analog);

	adc_channel_5(7); //conversion time 239.5 cycle
	adc_channel_7(7); //conversion time 239.5 cycle

	number_of_conversion(2); //number of conversion 2

	sequence_1(5); //bit 5 in first sequence
	sequence_2(7); //bit 7 in second sequence

	//adc DMA mode enable and scan
	enable_scan;
	adc_dma_mode;

	//dma settings
	DMA1_Channel1->CPAR = (uint32_t)(&(ADC1->DR));
	DMA1_Channel1->CMAR = (uint32_t)adcvalues;
	/*
	DMA1_Channel1->CNDTR = 2;
	DMA1_Channel1->CCR |= DMA_CCR1_CIRC | DMA_CCR1_MINC | DMA_CCR1_PSIZE_0 | DMA_CCR1_MSIZE_0;
	DMA1_Channel1->CCR |= DMA_CCR1_EN;
	*/
	dma1_init(circular_mode, halfword, halfword, minc_enabled, pinc_disabled, 2);
	//enable ADC for the first time and set continuous mode
	adc_on_continuous_mode;	//Turn ON ADC for the first time
	delay_ms(1);						// small delay to wake adc up
	adc_on; 								//turn ON ADC for the second time
	delay_ms(1);
	//calibration
	adc_calibration;
	while(ADC1->CR2 & ADC_CR2_CAL);

	delay_ms(2);
}

/*
void adc_init(void)
{
	RCC->CFGR |= RCC_CFGR_ADCPRE_DIV6; ////change ADC prescaler not to exceed 14 MHz
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN;
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	//enable end of conversion interrupt
	//ADC1->CR1 |= ADC_CR1_EOCIE;
	//enable interrupt in NVIC
	//NVIC_EnableIRQ(ADC1_2_IRQn);
	//set sampling rate
	ADC1->SMPR2 |= ADC_SMPR2_SMP5_2 | ADC_SMPR2_SMP5_1 | ADC_SMPR2_SMP5_0;
	ADC1->SMPR2 |= ADC_SMPR2_SMP7_2 | ADC_SMPR2_SMP7_1 | ADC_SMPR2_SMP7_0;
	//set the channel in sequence register
	ADC1->SQR3 |= ADC_SQR3_SQ1_0 | ADC_SQR3_SQ1_2;
	ADC1->SQR1 |= (1 << 20);
	ADC1->SQR3 |= (7 << 5);

	//adc DMA enable and scan
	enable_scanmode;
	ADC1->CR2 |= ADC_CR2_DMA;

	//dma settings
	DMA1_Channel1->CPAR = (uint32_t)(&(ADC1->DR));
	DMA1_Channel1->CMAR = (uint32_t)samples;
	DMA1_Channel1->CNDTR = 2;
	DMA1_Channel1->CCR |= DMA_CCR1_CIRC | DMA_CCR1_MINC | DMA_CCR1_PSIZE_0 | DMA_CCR1_MSIZE_0;
	DMA1_Channel1->CCR |= DMA_CCR1_EN;
	//enable ADC for the first time and set continuous mode
	ADC1->CR2 |= ADC_CR2_ADON | ADC_CR2_CONT;
	delay_ms(1);
	ADC1->CR2 |= ADC_CR2_ADON; //turn ON ADC for the second time
	delay_ms(1);
	//calibration
	ADC1->CR2 |= ADC_CR2_CAL;
	delay_ms(2);

}

*/

/*
******************************* ADC Example*******************************
void ADC1_2_IRQhandler(void)
{
	//check if weare hrer because of conversion flag is set
	//end of conversion flag is cleared by reading data from the register
	if(ADC1->SR & ADC_SR_EOC)
	{
		val = ADC1->DR;
	}


void ADC1_2_IRQHandler(void)
{
	if(adc_conversion_finished)
	{
		val = ADC1->DR;
	}
}

}*/


#endif /* MYADC_H_ */
