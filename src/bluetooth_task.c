#include "bluetooth_task.h"
#include "mygpio.h"
#include "stm32f10x.h"
#include <string.h>
#include <stdio.h>

/* Private variables */
TaskHandle_t xBluetoothTaskHandle = NULL;
QueueHandle_t xBluetoothQueue = NULL;
ring_buffer_t rx_buffer;
ring_buffer_t tx_buffer;

/* UART initialization */
void bluetooth_uart_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable clocks */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    /* Configure USART2 Tx (PA2) as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Configure USART2 Rx (PA3) as input floating */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* USART configuration */
    USART_InitStructure.USART_BaudRate = BLUETOOTH_BAUDRATE;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStructure);

    /* Enable USART2 Receive interrupt */
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    /* Configure NVIC for USART2 */
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Enable USART2 */
    USART_Cmd(USART2, ENABLE);
}

/* Send string over Bluetooth */
void bluetooth_send_string(const char *str)
{
    bluetooth_send_data((const uint8_t*)str, strlen(str));
}

/* Send data over Bluetooth */
void bluetooth_send_data(const uint8_t *data, uint32_t len)
{
    for(uint32_t i = 0; i < len; i++) {
        /* Wait until TX buffer is empty */
        while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
        USART_SendData(USART2, data[i]);
    }
}

/* Process received Bluetooth commands */
BluetoothCommand_t process_bluetooth_command(const char *command)
{
    if(strcmp(command, "LED_ON") == 0) {
        return BT_CMD_LED_ON;
    } else if(strcmp(command, "LED_OFF") == 0) {
        return BT_CMD_LED_OFF;
    } else if(strcmp(command, "LED_TOGGLE") == 0) {
        return BT_CMD_LED_TOGGLE;
    } else if(strcmp(command, "GET_ADC") == 0) {
        return BT_CMD_GET_ADC;
    } else if(strcmp(command, "STATUS") == 0) {
        return BT_CMD_GET_STATUS;
    } else {
        return BT_CMD_UNKNOWN;
    }
}

void bluetooth_task_init(void)
{
    /* Initialize ring buffers */
    ring_buffer_init(&rx_buffer);
    ring_buffer_init(&tx_buffer);

    /* Create Bluetooth queue */
    xBluetoothQueue = xQueueCreate(10, sizeof(BluetoothCommand_t));

    /* Initialize UART for Bluetooth */
    bluetooth_uart_init();

    /* Create Bluetooth task */
    xTaskCreate(vBluetoothTask,
                "Bluetooth Task",
                256,  // Larger stack for string processing
                NULL,
                2,    // Medium priority
                &xBluetoothTaskHandle);
}

void vBluetoothTask(void *pvParameters)
{
    uint8_t rx_byte;
    char command_buffer[32];
    uint8_t command_index = 0;
    BluetoothCommand_t bt_command;

    /* Send startup message */
    bluetooth_send_string("HC-05 Bluetooth Ready\r\n");
    bluetooth_send_string("Commands: LED_ON, LED_OFF, LED_TOGGLE, GET_ADC, STATUS\r\n");

    for(;;)
    {
        /* Process received bytes from ring buffer */
        while(ring_buffer_get(&rx_buffer, &rx_byte)) {
            /* Echo back (for testing) */
            bluetooth_send_data(&rx_byte, 1);

            if(rx_byte == '\r' || rx_byte == '\n') {
                /* End of command */
                if(command_index > 0) {
                    command_buffer[command_index] = '\0'; // Null terminate

                    /* Process command */
                    bt_command = process_bluetooth_command(command_buffer);
                    xQueueSend(xBluetoothQueue, &bt_command, 0);

                    command_index = 0;
                }
            } else if(command_index < sizeof(command_buffer) - 1) {
                /* Add to command buffer */
                command_buffer[command_index++] = rx_byte;
            }
        }

        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

/* USART2 Interrupt Handler */
void USART2_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
        /* Read received data */
        uint8_t data = USART_ReceiveData(USART2);

        /* Put data in ring buffer */
        ring_buffer_put(&rx_buffer, data);

        /* Clear interrupt flag */
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }

    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}
