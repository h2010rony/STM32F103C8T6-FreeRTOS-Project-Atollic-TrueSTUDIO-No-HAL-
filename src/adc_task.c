#include "adc_task.h"
#include "timers.h"
#include "stm32f10x.h"

/* Private variables */
TaskHandle_t xADCTaskHandle = NULL;
QueueHandle_t xADCQueue = NULL;

/* ADC initialization */
void adc_init(void)
{
    /* Enable ADC1 and GPIOA clocks */
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_IOPAEN;

    /* Configure PA0 as analog input */
    GPIOA->CRL &= ~(0xF << 0);      // Clear PA0 configuration
    GPIOA->CRL |= (0x00 << 0);      // Analog mode

    /* ADC configuration */
    ADC1->CR2 = 0;                  // Clear control register 2

    /* Power on ADC */
    ADC1->CR2 |= ADC_CR2_ADON;

    /* Calibration */
    ADC1->CR2 |= ADC_CR2_CAL;
    while(ADC1->CR2 & ADC_CR2_CAL); // Wait for calibration to complete

    /* Configure ADC */
    ADC1->CR2 |= ADC_CR2_CONT;      // Continuous conversion
    ADC1->SQR1 = 0;                 // 1 conversion in sequence
    ADC1->SQR3 = ADC_CHANNEL;       // Channel 0 (PA0)
    ADC1->SMPR2 = 0x07 << 0;        // 239.5 cycles sampling time for channel 0

    /* Start conversion */
    ADC1->CR2 |= ADC_CR2_ADON;      // Power on ADC again
    ADC1->CR2 |= ADC_CR2_SWSTART;   // Start conversion
}

uint16_t adc_read(uint8_t channel)
{
    /* Set channel */
    ADC1->SQR3 = channel;

    /* Start conversion */
    ADC1->CR2 |= ADC_CR2_SWSTART;

    /* Wait for conversion complete */
    while(!(ADC1->SR & ADC_SR_EOC));

    /* Read result */
    return ADC1->DR;
}

void adc_task_init(void)
{
    /* Create queue for ADC data */
    xADCQueue = xQueueCreate(10, sizeof(ADCData_t));

    /* Create ADC task */
    xTaskCreate(vADCTask,
                "ADC Task",
                128,  // Larger stack for ADC processing
                NULL,
                0,    // Same priority as LED task
                &xADCTaskHandle);
}

void vADCTask(void *pvParameters)
{
    ADCData_t adc_data;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    /* Initialize ADC */
    adc_init();

    for(;;)
    {
        /* Read ADC value */
        adc_data.raw_value = adc_read(ADC_CHANNEL);

        /* Convert to millivolts (3.3V reference, 12-bit ADC) */
        adc_data.voltage_mv = (adc_data.raw_value * 3300) / 4095;
        adc_data.channel = ADC_CHANNEL;

        /* Send data to queue (non-blocking) */
        xQueueSend(xADCQueue, &adc_data, 0);

        /* Print ADC values (you can replace this with other processing) */
        // In real implementation, you might send this over UART
        // or use it to control other tasks

        /* Wait for next sample */
        vTaskDelayUntil(&xLastWakeTime, ADC_SAMPLE_RATE_MS / portTICK_PERIOD_MS);
    }
}

