#ifndef ADC_TASK_H
#define ADC_TASK_H

#include "task_common.h"

/* Public functions */
void adc_task_init(void);
void vADCTask(void *pvParameters);
void adc_init(void);
uint16_t adc_read(uint8_t channel);

/* ADC configuration */
#define ADC_SAMPLE_RATE_MS   1000  // 1 second between samples
#define ADC_CHANNEL          0     // PA0 for ADC

#endif /* ADC_TASK_H */

