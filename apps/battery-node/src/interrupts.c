#include "interrupts.h"

#include "main.h"

extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;
extern CAN_HandleTypeDef hcan;

void DMA1_Channel1_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_adc1); }

void DMA2_Channel1_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_adc2); }

// USB high priority or CAN_TX interrupt handler.
void USB_HP_CAN_TX_IRQHandler(void) { HAL_CAN_IRQHandler(&hcan); }

// EXTI line[15:10] interrupt handler.
void EXTI15_10_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(OVER_CURRENT_Pin); }
