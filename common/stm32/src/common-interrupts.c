#include "common-interrupts.h"

#include "stm32f3xx_hal.h"

// FreeRTOS includes
#include "FreeRTOS.h"
#include "task.h"

// Non-maskable interrupt handler.
void NMI_Handler(void) {
  while (1) {
  }
}

// Hard fault handler.
void HardFault_Handler(void) {
  while (1) {
  }
}

// Memory management fault handler.
void MemManage_Handler(void) {
  while (1) {
  }
}

// Pre-fetch fault, memory access fault handler.
void BusFault_Handler(void) {
  while (1) {
  }
}

// Undefined instruction or illegal state fault handler.
void UsageFault_Handler(void) {
  while (1) {
  }
}

// Debug monitor handler.
void DebugMon_Handler(void) {}

// System tick timer handler.
void SysTick_Handler(void) {
  HAL_IncTick();
#if (INCLUDE_xTaskGetSchedulerState == 1)
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
#endif /* INCLUDE_xTaskGetSchedulerState */
    xPortSysTickHandler();
#if (INCLUDE_xTaskGetSchedulerState == 1)
  }
#endif /* INCLUDE_xTaskGetSchedulerState */
}
