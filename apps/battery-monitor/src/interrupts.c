#include "peripherals.h"
#include "ports.h"

void DMA1_Channel1_IRQHandler(void) {
  peripherals_t *peripherals = get_peripherals();
  HAL_DMA_IRQHandler(&peripherals->hdma_adc1);
}

void DMA2_Channel1_IRQHandler(void) {
  peripherals_t *peripherals = get_peripherals();
  HAL_DMA_IRQHandler(&peripherals->hdma_adc2);
}

void EXTI15_10_IRQHandler(void) {
  HAL_GPIO_EXTI_IRQHandler(OVER_CURRENT_PIN);
}
