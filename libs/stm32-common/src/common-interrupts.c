#include "common-interrupts.h"

#include "common-peripherals.h"
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
void DebugMon_Handler(void) {
}

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

void USART1_IRQHandler(void) {
  common_peripherals_t *common_peripherals = get_common_peripherals();
  HAL_UART_IRQHandler(&common_peripherals->huart1);
}

void USB_LP_CAN_RX0_IRQHandler(void) {
  common_peripherals_t *common_peripherals = get_common_peripherals();
  HAL_CAN_IRQHandler(&common_peripherals->hcan);
}
