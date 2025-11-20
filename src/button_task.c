#include "button_task.h"
#include "mygpio.h"
#include "timers.h"

/* Private variables */
TaskHandle_t xButtonTaskHandle = NULL;

void button_task_init(void)
{
    /* Create button task */
    xTaskCreate(vButtonTask,
                "Button Task",
                128,
                NULL,
                2,  // Higher priority than LED task
                &xButtonTaskHandle);
}

void vButtonTask(void *pvParameters)
{
    ButtonState_t last_button_state = BUTTON_RELEASED;
    ButtonState_t current_button_state;
    uint32_t button_press_count = 0;
    uint32_t last_press_time = 0;
    LEDCommand_t command_to_send;

    for(;;)
    {
        /* Read current button state */
        current_button_state = gpio_read(GPIOA, 0);  // Changed to GPIOA for button

        /* Detect button press (rising edge) */
        if((current_button_state == BUTTON_PRESSED) && (last_button_state == BUTTON_RELEASED))
        {
            uint32_t current_time = xTaskGetTickCount();

            /* Simple debounce - ignore presses too close together */
            if((current_time - last_press_time) > 200 / portTICK_PERIOD_MS)
            {
                button_press_count++;
                last_press_time = current_time;

                /* Cycle through LED modes on each button press */
                switch(button_press_count % 5)
                {
                    case 0:
                        command_to_send = LED_OFF;
                        break;
                    case 1:
                        command_to_send = LED_ON;
                        break;
                    case 2:
                        command_to_send = LED_BLINK_SLOW;
                        break;
                    case 3:
                        command_to_send = LED_BLINK_FAST;
                        break;
                    case 4:
                        command_to_send = LED_TOGGLE;
                        break;
                }

                /* Send command to LED task */
                xQueueSend(xLEDQueue, &command_to_send, 0);
            }
        }

        /* Update last button state */
        last_button_state = current_button_state;

        /* Delay for debouncing and to prevent CPU hogging */
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}
