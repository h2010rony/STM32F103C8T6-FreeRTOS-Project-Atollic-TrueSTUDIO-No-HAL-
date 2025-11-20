
#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE.
 *
 * See http://www.freertos.org/a00110.html
 *----------------------------------------------------------*/

#define INCLUDE_xTaskGetHandle                    0
#define configUSE_TRACE_FACILITY                  0

#define configUSE_PREEMPTION			1
#define configUSE_IDLE_HOOK				1
#define configUSE_TICK_HOOK				0
#define configCPU_CLOCK_HZ				( 72000000UL )
#define configTICK_RATE_HZ				( ( TickType_t ) 1000 )
#define configUSE_TICKLESS_IDLE         0
#define configMAX_PRIORITIES			( 3 ) //can be 5
#define configMINIMAL_STACK_SIZE		( ( unsigned short ) 70 ) //can be 64
#define configTOTAL_HEAP_SIZE			( ( size_t ) ( 6 * 1024 ) ) //can be 6x1024
#define configMAX_TASK_NAME_LEN			( 10 ) //can be 8
#define configUSE_TRACE_FACILITY		0
#define configUSE_16_BIT_TICKS			0
#define configIDLE_SHOULD_YIELD			1
#define configUSE_MUTEXES				0 //can be 1
#define configQUEUE_REGISTRY_SIZE		0 //can be 3
#define configUSE_QUEUE_SETS            0
#define configGENERATE_RUN_TIME_STATS	0
#define configCHECK_FOR_STACK_OVERFLOW	2
#define configUSE_RECURSIVE_MUTEXES		0
#define configUSE_MALLOC_FAILED_HOOK	0 //can be 1
#define configUSE_APPLICATION_TASK_TAG	0
#define configUSE_COUNTING_SEMAPHORES	0
#define configUSE_PORT_OPTIMISED_TASK_SELECTION    0
#define configUSE_TIME_SLICING                     0

/* Cortex-M Specific Definitions */
#define configPRIO_BITS                            4
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY    15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5


/* Memory allocation */
#define configSUPPORT_STATIC_ALLOCATION            0
#define configSUPPORT_DYNAMIC_ALLOCATION           1

/* Co-routine definitions. */
#define configUSE_CO_ROUTINES 		0
#define configMAX_CO_ROUTINE_PRIORITIES ( 2 )

/* Software timer definitions. */
#define configUSE_TIMERS				0 //can be 1
#define configTIMER_TASK_PRIORITY		( 3 )
#define configTIMER_QUEUE_LENGTH		5
#define configTIMER_TASK_STACK_DEPTH	( configMINIMAL_STACK_SIZE )

/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */
#define INCLUDE_vTaskPrioritySet		1
#define INCLUDE_uxTaskPriorityGet		1
#define INCLUDE_vTaskDelete				1
#define INCLUDE_vTaskCleanUpResources	1
#define INCLUDE_vTaskSuspend			1
#define INCLUDE_vTaskDelayUntil			1
#define INCLUDE_vTaskDelay				1
#define INCLUDE_xTaskGetTickCount       1
#define INCLUDE_xTaskGetCurrentTaskHandle          0                // Disabled
#define INCLUDE_xTaskGetSchedulerState             0                // Disabled
#define INCLUDE_uxTaskGetStackHighWaterMark        0
#define INCLUDE_xTaskGetHandle                    0
/* QUEUE FUNCTIONS - MUST BE ENABLED */
#define INCLUDE_xQueueCreate                       1                // For creating queues
#define INCLUDE_xQueueSend                         1                // For sending to queues
#define INCLUDE_xQueueSendToBack                   1                // Alternative send
#define INCLUDE_xQueueReceive                      1                // For receiving from queues
#define INCLUDE_xQueueGenericSend                  1                // Generic send
#define INCLUDE_xQueueGenericReceive               1                // Generic receive



/* Use the system definition, if there is one */
//#ifdef __NVIC_PRIO_BITS
//	#define configPRIO_BITS       __NVIC_PRIO_BITS
//#else
//	#define configPRIO_BITS       4        /* 15 priority levels */
//#endif


/* Interrupt configuration */
//#define configKERNEL_INTERRUPT_PRIORITY            (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
//#define configMAX_SYSCALL_INTERRUPT_PRIORITY       (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))


#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY			15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY	5

/* The lowest priority. */
//#define configKERNEL_INTERRUPT_PRIORITY 	( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )
/* Priority 5, or 95 as only the top four bits are implemented. */
/* !!!! configMAX_SYSCALL_INTERRUPT_PRIORITY must not be set to zero !!!!
See http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html. */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY 	( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )
	
//#define configASSERT( x ) if( ( x ) == 0 ) { taskDISABLE_INTERRUPTS(); for( ;; ); }
	
#define vPortSVCHandler 	SVC_Handler
#define xPortPendSVHandler 	PendSV_Handler
#define xPortSysTickHandler SysTick_Handler


/* Error checking */
#define configASSERT(x)


#endif /* FREERTOS_CONFIG_H */

