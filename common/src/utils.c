#include "utils.h"

#include <string.h>

#include "main.h"
#include "task.h"

extern UART_HandleTypeDef huart1;

// Print message to uart
void Print(char *str) {
  HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
}

void SignalTask(osThreadId_t taskHandle) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(taskHandle, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}