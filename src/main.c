#include <stdint.h>
#include <stddef.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "myclock.h"
#include "mygpio.h"
#include "stm32f10x.h"  // Standard library header
#include "task_common.h"  // Include the common header

/* Function prototypes */
static inline void gpio_init(void);
void process_adc_data(void);
void process_bluetooth_commands(void);  // New function


int main(void)
{
    /* System initialization */
    //SystemInit();  // From standard library
    setto72MHz();   // Your clock configuration
    gpio_init();

    /* Initialize all tasks */
    tasks_init();

    /* Start FreeRTOS scheduler */
    vTaskStartScheduler();

    /* Should never reach here */
    while(1){};
}

/**
 * @brief Hardware setup
 * Initializes all peripherals
 */
static inline void gpio_init(void)
{
    enable_portc_clock;
    config_gpio_output(GPIOC, 13, out_2MHz, GeneralPurpose);
    enable_porta_clock;
    config_gpio_input(GPIOA, 0, Floating);  // Button on PA0
}


void process_adc_data(void)
{
    ADCData_t adc_data;

    /* Check for new ADC data */
    if(xQueueReceive(xADCQueue, &adc_data, 0) == pdTRUE)
    {
        /* Example: Change LED behavior based on ADC value */
        if(adc_data.voltage_mv > 2000)
        {
            // High voltage detected - fast blink
            LEDCommand_t command = LED_BLINK_FAST;
            xQueueSend(xLEDQueue, &command, 0);
        }
        else if(adc_data.voltage_mv > 1000)
        {
            // Medium voltage - slow blink
            LEDCommand_t command = LED_BLINK_SLOW;
            xQueueSend(xLEDQueue, &command, 0);
        }
        else
        {
            // Low voltage - turn off
            LEDCommand_t command = LED_OFF;
            xQueueSend(xLEDQueue, &command, 0);
        }

        /* You could also send ADC data over UART here */
    }
}

void process_bluetooth_commands(void)
{
    BluetoothCommand_t bt_command;
    extern void bluetooth_send_string(const char *str);
    extern ADCData_t last_adc_reading;  // You'll need to track this in adc_task.c

    if(xQueueReceive(xBluetoothQueue, &bt_command, 0) == pdTRUE)
    {
        switch(bt_command)
        {
            case BT_CMD_LED_ON:
            {
                LEDCommand_t led_cmd = LED_ON;
                xQueueSend(xLEDQueue, &led_cmd, 0);
                bluetooth_send_string("LED turned ON\r\n");
                break;
            }
            case BT_CMD_LED_OFF:
            {
                LEDCommand_t led_cmd = LED_OFF;
                xQueueSend(xLEDQueue, &led_cmd, 0);
                bluetooth_send_string("LED turned OFF\r\n");
                break;
            }
            case BT_CMD_LED_TOGGLE:
            {
                LEDCommand_t led_cmd = LED_TOGGLE;
                xQueueSend(xLEDQueue, &led_cmd, 0);
                bluetooth_send_string("LED toggled\r\n");
                break;
            }
            case BT_CMD_GET_ADC:
            {
                char buffer[64];
                // You'll need to get the latest ADC reading from adc_task
                // snprintf(buffer, sizeof(buffer), "ADC: %d (%.2fV)\r\n",
                //         last_adc_reading.raw_value, last_adc_reading.voltage_mv / 1000.0);
                // bluetooth_send_string(buffer);
                bluetooth_send_string("ADC reading requested\r\n");
                break;
            }
            case BT_CMD_GET_STATUS:
                bluetooth_send_string("System status: OK\r\n");
                break;
            case BT_CMD_UNKNOWN:
                bluetooth_send_string("Unknown command\r\n");
                break;
        }
    }
}




/* FreeRTOS hook functions - PROPERLY TERMINATED */

#if (configUSE_MALLOC_FAILED_HOOK != 0)
void vApplicationMallocFailedHook( void )
{
    while(1)
    {
        gpio_toggle(GPIOC, 13);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
#endif  /* configUSE_MALLOC_FAILED_HOOK */

#if (configCHECK_FOR_STACK_OVERFLOW != 0)
void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName )
{
    (void)xTask;
    (void)pcTaskName;
    while(1)
    {
        gpio_toggle(GPIOC, 13);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
#endif  /* configCHECK_FOR_STACK_OVERFLOW */

#if (configUSE_IDLE_HOOK != 0)
void vApplicationIdleHook( void )
{
    /* Process ADC data in idle time */
    process_adc_data();

    /* Optional: Put CPU in low power mode */
    /* __WFI(); */
}
#endif  /* configUSE_IDLE_HOOK */






