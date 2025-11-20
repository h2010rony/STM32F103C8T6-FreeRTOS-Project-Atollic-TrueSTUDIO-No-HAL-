#ifndef TASK_COMMON_H
#define TASK_COMMON_H

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "stm32f10x.h"
#include "mygpio.h"
#include "led_task.h"
#include "button_task.h"
#include "adc_task.h"
#include "bluetooth_task.h"

/* Forward declarations */
extern QueueHandle_t xLEDQueue;
extern QueueHandle_t xADCQueue;  // New ADC queue
extern QueueHandle_t xBluetoothQueue;
extern TaskHandle_t xLEDTaskHandle;
extern TaskHandle_t xButtonTaskHandle;
extern TaskHandle_t xADCTaskHandle;  // New ADC task handle
extern TaskHandle_t xBluetoothTaskHandle;  // New Bluetooth task handle

/* Button states */
typedef enum {
    BUTTON_RELEASED = 0,
    BUTTON_PRESSED = 1
} ButtonState_t;

/* LED commands */
typedef enum {
    LED_OFF = 0,
    LED_ON = 1,
    LED_TOGGLE = 2,
    LED_BLINK_SLOW = 3,
    LED_BLINK_FAST = 4
} LEDCommand_t;

typedef struct {
    uint16_t raw_value;
    uint16_t voltage_mv;
    uint8_t channel;
} ADCData_t;


/* Bluetooth commands */
typedef enum {
    BT_CMD_LED_ON = 0,
    BT_CMD_LED_OFF,
    BT_CMD_LED_TOGGLE,
    BT_CMD_GET_ADC,
    BT_CMD_GET_STATUS,
    BT_CMD_UNKNOWN
} BluetoothCommand_t;


/* Task initialization function */
void tasks_init(void);

#endif /* TASK_COMMON_H */
