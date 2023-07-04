#include <string.h>

#include "sbus.h"
#include "steering.h"

// STM32Common
#include "clock.h"
#include "common-interrupts.h"
#include "error.h"
#include "flash.h"
#include "peripherals.h"
#include "print.h"
#include "stm32f3xx_hal.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

// Application
#define CAN_BASE_ID 0x100
#define CAN_MAX_DLC 8

typedef struct {
  uint32_t id;
  uint8_t dlc;
  uint8_t data[CAN_MAX_DLC];
} CANFrame;

// FreeRTOS
#define TASK_PRIORITY 24

static TaskHandle_t sbus_read_task;
static StaticTask_t sbus_read_buffer;
static StackType_t sbus_read_stack[configMINIMAL_STACK_SIZE];

void sbus_read(void *unused);

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

  sbus_read_task =
      xTaskCreateStatic(sbus_read, "sbus read", configMINIMAL_STACK_SIZE, NULL,
                        TASK_PRIORITY, sbus_read_stack, &sbus_read_buffer);

  print("Starting application...\r\n");

  // Start scheduler
  vTaskStartScheduler();

  // We should never get here as control is now taken by the scheduler
  while (1) {
  }
}

void sbus_read(void *unused) {
  (void)unused;

  peripherals_t *peripherals = get_peripherals();

  CANFrame frames[4];
  for (int i = 0; i < 4; i++) {
    frames[i].id = CAN_BASE_ID + i;
    frames[i].dlc = 2;
  }

  uint32_t mailbox = 0;

  uint8_t sbus_data[SBUS_PACKET_LENGTH];
  sbus_packet_t sbus_packet;
  steering_command_t steering_command;

  HAL_CAN_Start(&peripherals->common_peripherals->hcan);

  for (;;) {
    memset(sbus_data, 0, sizeof(sbus_data));

    // Wait until reception of one complete message, in case we power up in the
    // middle of a transmission
    while (sbus_data[0] != SBUS_HEADER) {
      HAL_UART_Receive(&peripherals->huart2, sbus_data, 1, 3);
    }

    HAL_UART_Receive_IT(&peripherals->huart2, &sbus_data[1],
                        sizeof(sbus_data) - 1);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    sbus_parse_data(sbus_data, &sbus_packet);
    steering_command = sbus_packet_to_steering_command(&sbus_packet);

    // Failsafe usually triggers if many frames are lost in a row
    // Indicates connection loss (heavy)
    if (sbus_packet.failsafe_activated) {
      print("Failsafe activated\r\n");
    }

    // Indicates slight connection loss or issue with frame.
    if (sbus_packet.frame_lost) {
      print("Frame lost\r\n");
    }

    memcpy(frames[0].data, &steering_command.steering, 2);
    memcpy(frames[1].data, &steering_command.steering_trim, 2);
    memcpy(frames[2].data, &steering_command.throttle, 2);
    memcpy(frames[3].data, &steering_command.throttle_trim, 2);
    for (int i = 0; i < 4; i++) {
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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  (void)huart;
  BaseType_t higher_priority_task_woken = pdFALSE;
  vTaskNotifyGiveFromISR(sbus_read_task, &higher_priority_task_woken);
  portYIELD_FROM_ISR(higher_priority_task_woken);
}
