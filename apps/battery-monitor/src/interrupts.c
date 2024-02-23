#include "battery.h"
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

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN) {
  if (GPIO_PIN != OVER_CURRENT_PIN) {
    return;
  }

  battery_state_t *battery_state = get_battery_state();
  battery_state->vbat_out.overcurrent_fault = true;
}
