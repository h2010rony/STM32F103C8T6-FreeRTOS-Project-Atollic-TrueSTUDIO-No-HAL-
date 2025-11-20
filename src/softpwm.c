/*
 * softpwm.c
 *
 *  Created on: Oct 27, 2025
 *      Author: Blu
 */

#include "softpwm.h"


volatile uint32_t pwm_duty = (PWM_RESOLUTION/2); /* default 50% */
volatile uint32_t pwm_counter = 0;

void gpio_pc13_init(void)
{
    /* Enable APB2 for I/O port C */
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;

    /* Configure PC13 as General purpose output push-pull, max speed 2 MHz.
       PC13 is in CRH (pins 8..15). For pin 13: CRH bits [ (13-8)*4 .. +3 ] */
    uint32_t pos = (13 - 8) * 4;
    /* Clear existing bits */
    GPIOC->CRH &= ~(0xF << pos);
    /* MODE13 = 10 (Output 2 MHz), CNF13 = 00 (General purpose push-pull) -> 0x2 */
    GPIOC->CRH |= (0x2 << pos);
}

/* TIM2 interrupt will be our PWM timebase.
   We configure TIM2 so its counter increments at 1 MHz (1us tick) if possible.
   Then ARR is calculated as timer_tick / (PWM_FREQ * PWM_RESOLUTION) - 1
*/
void tim2_init_for_pwm_timebase(void)
{
    /* Enable TIM2 clock (APB1) */
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    /* Compute prescaler to make timer tick = 1 MHz (1 microsecond tick)
       SystemCoreClock is provided by CMSIS (set by SystemInit/startup code).
       prescaler = SystemCoreClock/1_000_000 - 1 */
    uint32_t prescaler = (SystemCoreClock / 1000000U) - 1U;
    TIM2->PSC = prescaler;

    /* Compute ARR to achieve ISR frequency = PWM_FREQ_HZ * PWM_RESOLUTION */
    uint32_t isr_freq = PWM_FREQ_HZ * PWM_RESOLUTION; /* e.g. 200 * 200 = 40000 */
    if (isr_freq == 0) isr_freq = 1; /* safety */

    /* With timer tick = 1 MHz, ARR = (1_000_000 / isr_freq) - 1
       Check for overflow - ARR must fit 16-bit on TIM2? TIM2 is a 32-bit timer on many F1
       but to be safe check and cap */
    uint32_t arr = (1000000U / isr_freq);
    if (arr == 0) arr = 1;
    arr = arr - 1U;

    /* Minimal safety: if arr is too large for 32-bit (practically won't happen), clamp */
    if (arr > 0xFFFFFFFFU) arr = 0xFFFFFFFFU;

    TIM2->ARR = arr;

    /* Enable update interrupt */
    TIM2->DIER |= TIM_DIER_UIE;

    /* Enable counter */
    TIM2->CR1 |= TIM_CR1_CEN;

    /* Enable TIM2 IRQ in NVIC */
    NVIC_EnableIRQ(TIM2_IRQn);
    /* Optionally set priority */
    NVIC_SetPriority(TIM2_IRQn, 1);
}

/* Public API: set PWM duty [0 .. PWM_RESOLUTION] */
void pwm_set_duty_percent(uint32_t duty_steps)
{
    if (duty_steps > PWM_RESOLUTION) duty_steps = PWM_RESOLUTION;
    pwm_duty = duty_steps;
}

/* Example: convert percent (0..100) to steps */
void pwm_set_duty_percent_from_0_to_100(uint32_t percent)
{
    if (percent > 100) percent = 100;
    pwm_duty = (percent * PWM_RESOLUTION) / 100U;
}

/* TIM2 IRQ handler */
void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_UIF) /* update interrupt flag */
    {
        TIM2->SR &= ~TIM_SR_UIF; /* clear flag */

        /* Increment pwm counter */
        pwm_counter++;
        if (pwm_counter >= PWM_RESOLUTION) pwm_counter = 0;

        /* Output: PC13 = HIGH when pwm_counter < pwm_duty, else LOW
           Note: On many BluePill boards PC13 LED is active low (LED ON when PC13 = 0).
           This code toggles GPIO pin logically; adjust polarity in your wiring if needed. */
        if (pwm_counter < pwm_duty)
        {
            /* Set PC13 high */
            GPIOC->BSRR = (1U << 13);
        }
        else
        {
            /* Reset PC13 low */
            GPIOC->BRR = (1U << 13);
        }
    }
}

