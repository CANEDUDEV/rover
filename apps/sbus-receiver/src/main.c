#include <stm32f3xx_hal.h>
#include <stm32f3xx_hal_can.h>
#include <string.h>

#include "clock.h"
#include "common-interrupts.h"
#include "error.h"
#include "flash.h"
#include "peripherals.h"
#include "print.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

// Hardware
static peripherals_t *peripherals;

// Application
#define CAN_BASE_ID 100U
#define CAN_MAX_DLC 8

#define SBUS_PACKET_LENGTH 25
#define SBUS_HEADER 0x0F

typedef struct {
  uint32_t id;
  uint8_t dlc;
  uint8_t data[CAN_MAX_DLC];
} CANFrame;

// FreeRTOS
#define IO_READ_TASK_PERIOD_MS 20
#define TASK_PRIORITY 24

static TaskHandle_t sbus_read_task;
static StaticTask_t sbus_read_buffer;
static StackType_t sbus_read_stack[configMINIMAL_STACK_SIZE];

void sbus_read(void *argument);

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
  // Reset of all peripherals, Initializes the Flash interface and the Systick.
  HAL_Init();

  // Configure the system clock
  system_clock_init();

  // Initialize all configured peripherals
  peripherals_init();
  peripherals = get_peripherals();

  sbus_read_task =
      xTaskCreateStatic(sbus_read, "sbus read", configMINIMAL_STACK_SIZE, NULL,
                        TASK_PRIORITY, sbus_read_stack, &sbus_read_buffer);

  print(&peripherals->common_peripherals->huart1,
        "Starting application...\r\n");

  // Start scheduler
  vTaskStartScheduler();

  // We should never get here as control is now taken by the scheduler
  while (1) {
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  (void)huart;
  BaseType_t higher_priority_task_woken = pdFALSE;
  vTaskNotifyGiveFromISR(sbus_read_task, &higher_priority_task_woken);
  portYIELD_FROM_ISR(higher_priority_task_woken);
}

void sbus_read(void *argument) {
  UNUSED(argument);

  uint8_t sbusHeader = 0;
  uint8_t sbusData[SBUS_PACKET_LENGTH];

  peripherals_t *peripherals = get_peripherals();

  // Wait until reception of one complete message, in case we power up in the
  // middle of a transmission
  while (sbusHeader != SBUS_HEADER) {
    HAL_UART_Receive(&peripherals->huart2, &sbusHeader, sizeof(sbusHeader),
                     HAL_MAX_DELAY);
  }
  HAL_UART_Receive(&peripherals->huart2, sbusData, sizeof(sbusData) - 1,
                   HAL_MAX_DELAY);

  CANFrame frames[3];
  for (int i = 0; i < 3; i++) {
    frames[i].id = CAN_BASE_ID + i;
    frames[i].dlc = CAN_MAX_DLC;
  }

  uint32_t mailbox = 0;

  HAL_CAN_Start(&peripherals->common_peripherals->hcan);
  for (;;) {
    memset(sbusData, 0, sizeof(sbusData));
    HAL_UART_Receive_IT(&peripherals->huart2, sbusData, sizeof(sbusData));
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    for (int i = 0; i < 3; i++) {
      memcpy(frames[i].data, &sbusData[i * CAN_MAX_DLC], CAN_MAX_DLC);
      while (HAL_CAN_GetTxMailboxesFreeLevel(
                 &peripherals->common_peripherals->hcan) < 1) {
      }

      CAN_TxHeaderTypeDef header = {
          .StdId = frames[i].id,
          .DLC = frames[i].dlc,
      };

      HAL_CAN_AddTxMessage(&peripherals->common_peripherals->hcan, &header,
                           frames[i].data, &mailbox);
    }
  }
}
