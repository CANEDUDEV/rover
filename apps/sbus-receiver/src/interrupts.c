
#include "peripherals.h"
#include "stm32f3xx_hal.h"

void USART2_IRQHandler(void) {
  peripherals_t *peripherals = get_peripherals();
  HAL_UART_IRQHandler(&peripherals->huart2);
}
