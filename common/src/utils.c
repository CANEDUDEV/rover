#include "utils.h"

#include <string.h>

#include "main.h"

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
extern UART_HandleTypeDef huart1;

// Print message to uart
void Print(char *str) {
  HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
}
