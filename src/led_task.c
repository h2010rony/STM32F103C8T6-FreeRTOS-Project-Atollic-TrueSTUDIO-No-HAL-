#include "led_task.h"
#include "mygpio.h"
#include "timers.h"

/* Private variables */
TaskHandle_t xLEDTaskHandle = NULL;
QueueHandle_t xLEDQueue = NULL;

void led_task_init(void)
{
    /* Create queue for LED commands */
    xLEDQueue = xQueueCreate(5, sizeof(LEDCommand_t));

    /* Create LED task */
    xTaskCreate(vLEDTask,
                "LED Task",
                128,
                NULL,
                1,
                &xLEDTaskHandle);
}

void vLEDTask(void *pvParameters)
{
    LEDCommand_t xReceivedCommand;
    TickType_t xLastWakeTime;
    uint8_t current_mode = LED_BLINK_SLOW; // Default mode

    xLastWakeTime = xTaskGetTickCount();

    for(;;)
    {
        /* Check for new commands from button task */
        if(xQueueReceive(xLEDQueue, &xReceivedCommand, 0) == pdTRUE)
        {
            current_mode = xReceivedCommand;

            /* Execute immediate commands */
            switch(xReceivedCommand)
            {
                case LED_OFF:
                    gpio_clear(GPIOC, 13);
                    break;
                case LED_ON:
                    gpio_set(GPIOC, 13);
                    break;
                case LED_TOGGLE:
                    gpio_toggle(GPIOC, 13);
                    break;
                default:
                    /* For blink modes, just update the mode */
                    break;
            }
        }

        /* Handle continuous modes */
        switch(current_mode)
        {
            case LED_BLINK_SLOW:
                /* Blink every 1000ms */
                if((xTaskGetTickCount() - xLastWakeTime) >= 1000 / portTICK_PERIOD_MS)
                {
                    gpio_toggle(GPIOC, 13);
                    xLastWakeTime = xTaskGetTickCount();
                }
                break;

            case LED_BLINK_FAST:
                /* Blink every 250ms */
                if((xTaskGetTickCount() - xLastWakeTime) >= 250 / portTICK_PERIOD_MS)
                {
                    gpio_toggle(GPIOC, 13);
                    xLastWakeTime = xTaskGetTickCount();
                }
                break;

            case LED_OFF:
            case LED_ON:
                /* LED state is fixed, no blinking needed */
                break;

            default:
                break;
        }

        /* Small delay to prevent task from hogging CPU */
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

