/*
 * myi2c.h
 *
 *  Created on: May 11, 2024
 *      Author: Blu
 */

#ifndef MYI2C_H_
#define MYI2C_H_


#define EEPROM1	0b10100000

#define bufflen 20
uint8_t i2c_buff[bufflen];

void i2c2_init(void);
void i2c_WriteSingleByte(uint8_t device_address, uint8_t mem_address, uint8_t data);
void i2c_read(uint8_t device_address, uint8_t length);



void i2c2_init(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	I2C2->CR2 = 36; //setting clock
	I2C2->CCR = 180; //speed to 100 KHz
	I2C2->TRISE = 37;
	I2C2->CR1 |= I2C_CR1_ACK; //enable ack
  //alternate function output open drain
	GPIOB->CRH |= GPIO_CRH_CNF10 | GPIO_CRH_MODE10 | GPIO_CRH_CNF11 | GPIO_CRH_MODE11;
	I2C2->CR1 |= I2C_CR1_PE;
}

void i2c_WriteSingleByte(uint8_t dev_address, uint8_t mem_address, uint8_t data)
{
	uint32_t tempdata;
	I2C2->CR1 |= I2C_CR1_START; //generate start condition
	while(!(I2C2->SR1 & I2C_SR1_SB));
	I2C2->DR = dev_address;
	while(!(I2C2->SR1 & I2C_SR1_ADDR));
	tempdata = I2C2->SR2;
	I2C2->DR = mem_address;
	while(!(I2C2->SR1 & I2C_SR1_TXE));
	I2C2->DR = data;

	while(!(I2C2->SR1 & I2C_SR1_TXE));
	I2C2->CR1 |= I2C_CR1_STOP;
}

void i2c_read(uint8_t device_address, uint8_t length)
{
	uint32_t temp = 0;
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	I2C2->CR2 |= I2C_CR2_DMAEN;
	I2C2->CR1 |= I2C_CR1_ACK; //enable ack
	DMA1_Channel5->CMAR = (uint32_t)i2c_buff;
	DMA1_Channel5->CPAR = (uint32_t)I2C2->DR;
	DMA1_Channel5->CNDTR = length;
	DMA1_Channel5->CCR |= DMA_CCR4_TCIE | DMA_CCR5_MINC | DMA_CCR5_EN;
	I2C2->CR1 |= I2C_CR1_START; //generate start bit
	while(!(I2C2->SR1 & I2C_SR1_SB));
	I2C2->DR = device_address+1;
	while(!(I2C2->SR1 & I2C_SR1_ADDR));
	temp = I2C2->SR2;
	while((DMA1->ISR & DMA_ISR_TCIF5)==0);
	I2C2->CR1 |= I2C_CR1_STOP;
}






#endif /* MYI2C_H_ */
