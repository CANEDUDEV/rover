#include "interrupts.h"

#include "peripherals.h"
#include "stm32f3xx_hal.h"

// USB high priority or CAN_TX interrupt handler.
void USB_HP_CAN_TX_IRQHandler(void) {
  peripherals_t *peripherals = get_peripherals();
  HAL_CAN_IRQHandler(&peripherals->hcan);
}

void USART1_IRQHandler(void) {
  peripherals_t *peripherals = get_peripherals();
  HAL_UART_IRQHandler(&peripherals->huart1);
}
