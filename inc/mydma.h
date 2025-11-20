/*
 * mydma.h
 *
 *  Created on: May 11, 2024
 *      Author: Blu
 */

#ifndef MYDMA_H_
#define MYDMA_H_

#include <stdint.h>
#include "stm32f10x.h"
/*Memory and peripheral size*/
#define	byte			0
#define halfword	1
#define word			2

/*Mode */
#define non_circular_mode	0
#define minc_disabled			0
#define pinc_disabled			0
#define minc_enabled			1
#define pinc_enabled			1
#define circular_mode			1





/*DMA Channel Configuration Register DMAx->CCRx*/
#define enable_dma1_channel1	DMA1_Channel1->CCR |= (1 << 0)


void dma1_init(uint32_t dma_mode, uint32_t memory_size, uint32_t peripheral_size, uint32_t minc_mode, uint32_t pinc_mode, uint32_t numberof_data_register);



void dma1_init(uint32_t dma_mode, uint32_t memory_size, uint32_t peripheral_size, uint32_t minc_mode, uint32_t pinc_mode, uint32_t numberof_data_register)
{
	DMA1_Channel1->CNDTR = numberof_data_register;
	DMA1_Channel1->CCR |= (memory_size << 10); //memory size
	DMA1_Channel1->CCR |= (peripheral_size << 8); //memory size
	DMA1_Channel1->CCR |= (minc_mode << 7); //memory increment mode
	DMA1_Channel1->CCR |= (pinc_mode << 6); // peripheral increment mode
	DMA1_Channel1->CCR |= (dma_mode << 5);
	enable_dma1_channel1;
}





#endif /* MYDMA_H_ */
