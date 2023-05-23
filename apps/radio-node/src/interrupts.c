#include "interrupts.h"

#include "stm32f3xx_hal.h"

extern CAN_HandleTypeDef hcan;
extern UART_HandleTypeDef huart1;

// USB high priority or CAN_TX interrupt handler.
void USB_HP_CAN_TX_IRQHandler(void) { HAL_CAN_IRQHandler(&hcan); }

// UART1 interrupt handler.
void USART1_IRQHandler(void) { HAL_UART_IRQHandler(&huart1); }
