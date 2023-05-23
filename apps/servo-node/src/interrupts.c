#include "interrupts.h"

#include "stm32f3xx_hal.h"

extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;
extern CAN_HandleTypeDef hcan;

void DMA1_Channel1_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_adc1); }

void DMA2_Channel1_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_adc2); }

// USB high priority or CAN_TX interrupt handler.
void USB_HP_CAN_TX_IRQHandler(void) { HAL_CAN_IRQHandler(&hcan); }
