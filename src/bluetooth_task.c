#include "bluetooth_task.h"
#include "mygpio.h"
#include "stm32f10x.h"
#include <string.h>

/* Private variables */
TaskHandle_t xBluetoothTaskHandle = NULL;
QueueHandle_t xBluetoothQueue = NULL;
ring_buffer_t rx_buffer;

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
    while(*str) {
        while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
        USART_SendData(USART2, *str++);
    }
}

/* Process received Bluetooth commands */
BluetoothCommand_t process_bluetooth_command(const char *command)
{
    /* Single character commands to save RAM */
    if(strcmp(command, "1") == 0) return BT_CMD_LED_ON;
    if(strcmp(command, "0") == 0) return BT_CMD_LED_OFF;
    if(strcmp(command, "T") == 0) return BT_CMD_LED_TOGGLE;
    if(strcmp(command, "A") == 0) return BT_CMD_GET_ADC;
    if(strcmp(command, "S") == 0) return BT_CMD_GET_STATUS;
    return BT_CMD_UNKNOWN;
}

void bluetooth_task_init(void)
{
    /* Initialize RX ring buffer */
    ring_buffer_init(&rx_buffer);

    /* Create Bluetooth queue */
    xBluetoothQueue = xQueueCreate(5, sizeof(BluetoothCommand_t));

    /* Initialize UART */
    bluetooth_uart_init();

    /* Create Bluetooth task with optimized stack */
    xTaskCreate(vBluetoothTask,
                "BT",
                128,
                NULL,
                2,
                &xBluetoothTaskHandle);
}

void vBluetoothTask(void *pvParameters)
{
    uint8_t rx_byte;
    char command_buffer[16];  // Smaller buffer
    uint8_t command_index = 0;
    BluetoothCommand_t bt_command;

    /* Send startup message */
    bluetooth_send_string("BT Ready\r\n");
    bluetooth_send_string("1=ON,0=OFF,T=Toggle,A=ADC,S=Status\r\n");

    for(;;)
    {
        /* Process received bytes */
        while(ring_buffer_get(&rx_buffer, &rx_byte)) {
            /* Echo back */
            while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
            USART_SendData(USART2, rx_byte);

            if(rx_byte == '\r' || rx_byte == '\n') {
                if(command_index > 0) {
                    command_buffer[command_index] = '\0';
                    bt_command = process_bluetooth_command(command_buffer);
                    xQueueSend(xBluetoothQueue, &bt_command, 0);
                    command_index = 0;
                }
            } else if(command_index < sizeof(command_buffer) - 1) {
                command_buffer[command_index++] = rx_byte;
            }
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

/* USART2 Interrupt Handler */
void USART2_IRQHandler(void)
{
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
        uint8_t data = USART_ReceiveData(USART2);
        ring_buffer_put(&rx_buffer, data);
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}
