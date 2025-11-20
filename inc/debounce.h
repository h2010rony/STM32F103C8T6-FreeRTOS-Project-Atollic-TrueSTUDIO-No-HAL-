/*
 * debounce.h
 *
 *  Created on: May 11, 2024
 *      Author: Blu
 */

#ifndef DEBOUNCE_H_
#define DEBOUNCE_H_


#define  buttonpressedthresholdlevel 3000

static unsigned char  button_pressed_state = 0;
static uint16_t	buttonpressconfidencelevel=0;
static uint16_t   buttonreleaseconfidencelevel=0;
static unsigned char  LEDState=0;


void debounce_init(GPIO_TypeDef *port, uint32_t buttonno);
unsigned char buttondebounce(GPIO_TypeDef *port, uint32_t buttonno);
void debounce_out(GPIO_TypeDef *port, uint32_t outputpin, unsigned char returnval);

void debounce_init(GPIO_TypeDef *port, uint32_t buttonno)
{
	config_gpio_input(port, buttonno, PullUpPullDown);
}


unsigned char buttondebounce(GPIO_TypeDef *port, uint32_t buttonno)
{
	if((pin_is_set(port, buttonno)))
		{
			//if button_pressed_state is 0 then it will enter into this
			if(button_pressed_state==0)
			{
				//Increment the press confidence level
				buttonpressconfidencelevel++;
				//reinitialize the release confidence level
				buttonreleaseconfidencelevel=0;
				//if press level is more than threshold level
				if(buttonpressconfidencelevel>=buttonpressedthresholdlevel)
				{
					//toggle LED

					if(LEDState==0)
					{
						LEDState=1;

						//turn on the LED
						//set_bit(GPIOC, 13);
					}
					else
					{
						LEDState=0;
						//turn off the LED
						//clear_bit(GPIOC, 13);
					}
					//update button pressed value to 1
					button_pressed_state=1;

				}
				else
				{
					buttonpressconfidencelevel++;
					buttonreleaseconfidencelevel=0;
				}

			}


		}

	  else
		{

			if(button_pressed_state==1)
			{
				buttonreleaseconfidencelevel++;
				buttonpressconfidencelevel=0;
				if(buttonreleaseconfidencelevel>=buttonpressedthresholdlevel)
				{
					//update button pressed value to 0
					button_pressed_state=0;
				}
				else
				{
					buttonreleaseconfidencelevel++;
					buttonpressconfidencelevel=0;
				}

			}

		}
		return LEDState;
}


void debounce_out(GPIO_TypeDef *port, uint32_t outputpin, unsigned char returnval)
{
	if(returnval) set_bit(port, outputpin);
		else clear_bit(port, outputpin);
}







#endif /* DEBOUNCE_H_ */
