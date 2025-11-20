
#include "mygpio.h"

/*Pin position array*/
static uint32_t pinpos[16] = { 0x00, 0x04, 0x08, 0x0C, 0x10, 0x14, 0x18, 0x1C, 0x00, 0x04, 0x08, 0x0C, 0x10, 0x14, 0x18, 0x1C };



void config_gpio_input(GPIO_TypeDef *port, uint32_t PinNumber, uint32_t mode_type)
{

	if(PinNumber >= 8)
	{

		port->CRH &= ~((1 << pinpos[PinNumber]) | (1 << (pinpos[PinNumber]+1)));

		switch(mode_type)
		{
				case Analog					:		port->CRH &= ~((1<<CNF_BIT1) | (1<<CNF_BIT2));
																break;

				case Floating				:		port->CRH &= ~(1<<CNF_BIT2);
																port->CRH |= (1<<CNF_BIT1);
																break;

				case PullUpPullDown 		:		port->CRH |= (1<<CNF_BIT2);
																port->CRH &= ~(1<<CNF_BIT1);
																break;
		}

	}
	else
	{

			port->CRL &= ~((1 << pinpos[PinNumber]) | (1 << (pinpos[PinNumber]+1)));

			switch(mode_type)
			{
				case Analog			:	port->CRL &= ~((1<<CNF_BIT1) | (1<<CNF_BIT2));
										break;

				case Floating		:	port->CRL &= ~(1<<CNF_BIT2);
										port->CRL |= (1<<CNF_BIT1);
										break;

				case PullUpPullDown :	port->CRL |= (1<<CNF_BIT2);
										port->CRL &= ~(1<<CNF_BIT1);
										break;
			}

	}
}


void config_gpio_output(GPIO_TypeDef *port, uint32_t PinNumber, uint32_t speed, uint32_t mode_type)
{

	if(PinNumber >= 8)
	{
		port->CRH |= (speed << pinpos[PinNumber]);

		switch(mode_type)
		{
			case GeneralPurpose					:	port->CRH &= ~((1<<CNF_BIT1) | (1<<CNF_BIT2));
													break;

			case OpenDrain						:	port->CRH &= ~(1<<CNF_BIT2);
													port->CRH |= (1<<CNF_BIT1);
													break;

			case AlternateFunction				:	port->CRH |= (1<<CNF_BIT2);
													port->CRH &= ~(1<<CNF_BIT1);
													break;

			case AlternateFunction_OpenDrain	:	port->CRH |= (1<<CNF_BIT2);
													port->CRH |= (1<<CNF_BIT1);
													break;
		}
	}

	else
	{
		port->CRL |= (speed << pinpos[PinNumber]);

		switch(mode_type)
		{
			case GeneralPurpose					:	port->CRL &= ~((1<<CNF_BIT1) | (1<<CNF_BIT2));
													break;

			case OpenDrain						:	port->CRL &= ~(1<<CNF_BIT2);
													port->CRL |= (1<<CNF_BIT1);
													break;

			case AlternateFunction				:	port->CRL |= (1<<CNF_BIT2);
													port->CRL &= ~(1<<CNF_BIT1);
													break;

			case AlternateFunction_OpenDrain	:	port->CRL |= (1<<CNF_BIT2);
													port->CRL |= (1<<CNF_BIT1);
													break;
		}
	}

}


void config_gpio_interrupt(GPIO_TypeDef *port, uint32_t PinNumber, edge_select edge)
{
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;

	if(port == GPIOA)
	{
		switch(PinNumber)
		{
			case 0: AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI0_PA;   break;
			case 1: AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI1_PA;   break;
			case 2: AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI2_PA;   break;
			case 3: AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI3_PA;   break;
			case 4: AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI4_PA;   break;
			case 5: AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI5_PA;   break;
			case 6: AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI6_PA;   break;
			case 7: AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI7_PA;   break;
			case 8: AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI8_PA;   break;
			case 9: AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI9_PA;   break;
			case 10: AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI10_PA; break;
			case 11: AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI11_PA; break;
			case 12: AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI12_PA; break;
			case 13: AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI13_PA; break;
			case 14: AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI14_PA; break;
			case 15: AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI15_PA; break;
		}
	}

	if(port == GPIOB)
	{
		switch(PinNumber)
		{
			case 0: AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI0_PB;   break;
			case 1: AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI1_PB;   break;
			case 2: AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI2_PB;   break;
			case 3: AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI3_PB;   break;
			case 4: AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI4_PB;   break;
			case 5: AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI5_PB;   break;
			case 6: AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI6_PB;   break;
			case 7: AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI7_PB;   break;
			case 8: AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI8_PB;   break;
			case 9: AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI9_PB;   break;
			case 10: AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI10_PB; break;
			case 11: AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI11_PB; break;
			case 12: AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI12_PB; break;
			case 13: AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI13_PB; break;
			case 14: AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI14_PB; break;
			case 15: AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI15_PB; break;
		}
	}

	if(port == GPIOC)
	{
		switch(PinNumber)
		{
			case 0: AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI0_PC;   break;
			case 1: AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI1_PC;   break;
			case 2: AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI2_PC;   break;
			case 3: AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI3_PC;   break;
			case 4: AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI4_PC;   break;
			case 5: AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI5_PC;   break;
			case 6: AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI6_PC;   break;
			case 7: AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI7_PC;   break;
			case 8: AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI8_PC;   break;
			case 9: AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI9_PC;   break;
			case 10: AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI10_PC; break;
			case 11: AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI11_PC; break;
			case 12: AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI12_PC; break;
			case 13: AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI13_PC; break;
			case 14: AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI14_PC; break;
			case 15: AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI15_PC; break;
		}
	}

	if(port == GPIOD)
	{
		switch(PinNumber)
		{
			case 0: AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI0_PD;   break;
			case 1: AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI1_PD;   break;
			case 2: AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI2_PD;   break;
			case 3: AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI3_PD;   break;
			case 4: AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI4_PD;   break;
			case 5: AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI5_PD;   break;
			case 6: AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI6_PD;   break;
			case 7: AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI7_PD;   break;
			case 8: AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI8_PD;   break;
			case 9: AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI9_PD;   break;
			case 10: AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI10_PD; break;
			case 11: AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI11_PD; break;
			case 12: AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI12_PD; break;
			case 13: AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI13_PD; break;
			case 14: AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI14_PD; break;
			case 15: AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI15_PD; break;
		}
	}

	if(edge == RisingEdge) EXTI->RTSR |= (1 << PinNumber);
	if(edge == FallingEdge) EXTI->FTSR |= (1 << PinNumber);
	if(edge == RisingFallingEdge)
	{
		EXTI->RTSR |= (1 << PinNumber);
		EXTI->FTSR |= (1 << PinNumber);
	}

}


void enable_gpio_interrupt(uint32_t PinNumber, IRQn_Type irqNumber)
{
	//ENABLE interrupt in EXTI
	EXTI->IMR |= (1 << PinNumber);
	//Enable interrupt in NVIC
	NVIC_EnableIRQ(irqNumber);
}

void gpio_write(GPIO_TypeDef *port, uint32_t pinnumber, uint32_t PinState)
{
	if(PinState)
	{
		port->BSRR |= (1 << pinnumber);
	}
	else
	{
		port->BSRR |= (1 << (pinnumber+16));
	}
}


uint8_t gpio_read(GPIO_TypeDef *port, uint32_t pinnumber)
{
    return (port->IDR & (1 << pinnumber)) ? 0 : 1; //if high pin read value 0
}


void gpio_set(GPIO_TypeDef *port, uint32_t pinnumber)
{
	port->BSRR |= (1 << pinnumber);
}


void gpio_clear(GPIO_TypeDef *port, uint32_t pinnumber)
{
	port->BRR |= (1 << pinnumber);
}


void gpio_toggle(GPIO_TypeDef *port, uint32_t pinnumber)
{
	if(port->ODR & (1 << pinnumber)) port->BRR |= (1 << pinnumber); //pin OFF
	else port->BSRR |= (1 << pinnumber); //pin ON
}

