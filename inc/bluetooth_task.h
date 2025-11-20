#ifndef BLUETOOTH_TASK_H
#define BLUETOOTH_TASK_H

#include "task_common.h"
#include "ring_buffer.h"

/* HC-05 Bluetooth module configuration */
#define BLUETOOTH_BAUDRATE      9600
#define BLUETOOTH_USART         USART2
#define BLUETOOTH_IRQn          USART2_IRQn

/* Bluetooth commands */
typedef enum {
    BT_CMD_LED_ON = 0,
    BT_CMD_LED_OFF,
    BT_CMD_LED_TOGGLE,
    BT_CMD_GET_ADC,
    BT_CMD_GET_STATUS,
    BT_CMD_UNKNOWN
} BluetoothCommand_t;

/* Public functions */
void bluetooth_task_init(void);
void vBluetoothTask(void *pvParameters);
void bluetooth_send_string(const char *str);
void bluetooth_send_data(const uint8_t *data, uint32_t len);
void USART2_IRQHandler(void);

#endif /* BLUETOOTH_TASK_H */
