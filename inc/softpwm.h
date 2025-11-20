#ifndef SOFTPWM_H_
#define SOFTPWM_H_

#include "stm32f10x.h"

#define PWM_FREQ_HZ       200U     /* Desired PWM frequency in Hz (e.g. 200Hz) */
#define PWM_RESOLUTION    200U     /* Number of discrete steps (resolution). 0..(RES-1) */
                                  /* Keep RES*PWM_FREQ reasonably small to fit TIM ARR limits */

/* Globals for PWM */




void gpio_pc13_init(void);
void tim2_init_for_pwm_timebase(void);
void pwm_set_duty_percent(uint32_t duty_steps);
void pwm_set_duty_percent_from_0_to_100(uint32_t percent);
void TIM2_IRQHandler(void);


#endif


