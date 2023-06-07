#include "print.h"

#include <string.h>

void print(UART_HandleTypeDef *huart, char *str) {
  HAL_UART_Transmit_IT(huart, (uint8_t *)str, strlen(str));
}
