#include "peripherals.h"

void DMA1_Channel2_IRQHandler(void) {
  peripherals_t *peripherals = get_peripherals();
  HAL_DMA_IRQHandler(&peripherals->hdma_tim1_ch1);
}

void DMA1_Channel3_IRQHandler(void) {
  peripherals_t *peripherals = get_peripherals();
  HAL_DMA_IRQHandler(&peripherals->hdma_tim1_ch2);
}

void DMA1_Channel6_IRQHandler(void) {
  peripherals_t *peripherals = get_peripherals();
  HAL_DMA_IRQHandler(&peripherals->hdma_tim1_ch3);
}

void DMA1_Channel4_IRQHandler(void) {
  peripherals_t *peripherals = get_peripherals();
  HAL_DMA_IRQHandler(&peripherals->hdma_tim1_ch4);
}

void USART2_IRQHandler(void) {
  peripherals_t *peripherals = get_peripherals();
  HAL_UART_IRQHandler(&peripherals->huart2);
}
