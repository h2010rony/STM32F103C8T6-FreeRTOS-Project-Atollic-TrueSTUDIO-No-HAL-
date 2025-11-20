#ifndef TASK_COMMON_H
#define TASK_COMMON_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Forward declarations */
extern QueueHandle_t xLEDQueue;
extern QueueHandle_t xADCQueue;
extern QueueHandle_t xBluetoothQueue;
extern TaskHandle_t xLEDTaskHandle;
extern TaskHandle_t xButtonTaskHandle;
extern TaskHandle_t xADCTaskHandle;
extern TaskHandle_t xBluetoothTaskHandle;

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

/* ADC data structure */
typedef struct {
    uint16_t raw_value;
    uint16_t voltage_mv;
    uint8_t channel;
} ADCData_t;

/* Bluetooth commands - ONLY DEFINED HERE */
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
