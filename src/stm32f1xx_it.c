#include "stm32f1xx_it.h"

/* External declarations for FreeRTOS functions */
extern void xPortPendSVHandler(void);
extern void xPortSysTickHandler(void);
extern void vPortSVCHandler(void);

/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  while (1)
  {
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  while (1)
  {
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  while (1)
  {
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  while (1)
  {
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
// REMOVE SVC_Handler - FreeRTOS provides it
// void SVC_Handler(void)
// {
// }

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief This function handles Pendable request for system service.
  */
// REMOVE PendSV_Handler - FreeRTOS provides it
// void PendSV_Handler(void)
// {
// }

/**
  * @brief This function handles System tick timer.
  */
// REMOVE SysTick_Handler - FreeRTOS provides it
// void SysTick_Handler(void)
// {
// }

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
