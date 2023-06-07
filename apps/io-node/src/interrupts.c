#include "interrupts.h"

#include "peripherals.h"
#include "stm32f3xx_hal.h"

void DMA1_Channel1_IRQHandler(void) {
  peripherals_t *peripherals = get_peripherals();
  HAL_DMA_IRQHandler(&peripherals->hdma_adc1);
}

void USART1_IRQHandler(void) {
  peripherals_t *peripherals = get_peripherals();
  HAL_UART_IRQHandler(&peripherals->huart1);
}
