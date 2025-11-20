#include "task_common.h"
#include "led_task.h"
#include "button_task.h"
#include "bluetooth_task.h"

/* Global variables definition */


void tasks_init(void)
{
    /* Initialize all tasks */
    led_task_init();
    button_task_init();
    adc_task_init();
    bluetooth_task_init();  // Initialize Bluetooth task
}

