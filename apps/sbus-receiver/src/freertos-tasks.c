#include "freertos-tasks.h"

#include <string.h>

#include "ck-data.h"
#include "sbus.h"
#include "steering.h"

// CK
#include "mayor.h"
#include "postmaster-hal.h"

// STM32Common
#include "error.h"
#include "peripherals.h"
#include "print.h"
#include "printf.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"

#define LOWEST_TASK_PRIORITY 24

static TaskHandle_t sbus_read_task;
static StaticTask_t sbus_read_buf;
static StackType_t sbus_read_stack[configMINIMAL_STACK_SIZE];

// CAN Kingdom process received letters task
static TaskHandle_t process_letter_task;
static StaticTask_t process_letter_buf;
static StackType_t process_letter_stack[configMINIMAL_STACK_SIZE];

void sbus_read(void *unused);
// SBUS helpers
int sbus_read_header(uint8_t *sbus_data);
int sbus_read_data(uint8_t *sbus_data);
void send_steering_command(steering_command_t *command);

void process_letter(void *unused);
void dispatch_letter(ck_letter_t *letter);

void task_init(void) {
  sbus_read_task =
      xTaskCreateStatic(sbus_read, "sbus read", configMINIMAL_STACK_SIZE, NULL,
                        LOWEST_TASK_PRIORITY, sbus_read_stack, &sbus_read_buf);

  process_letter_task = xTaskCreateStatic(
      process_letter, "process letter", configMINIMAL_STACK_SIZE, NULL,
      LOWEST_TASK_PRIORITY + 1, process_letter_stack, &process_letter_buf);
}

void sbus_read(void *unused) {
  (void)unused;

  peripherals_t *peripherals = get_peripherals();

  uint8_t sbus_data[SBUS_PACKET_LENGTH];
  sbus_packet_t sbus_packet;
  steering_command_t steering_command;

  // Clear receive buffer before we start
  __HAL_UART_FLUSH_DRREGISTER(&peripherals->huart2);

  for (;;) {
    memset(sbus_data, 0, sizeof(sbus_data));

    if (sbus_read_header(sbus_data) != APP_OK) {
      steering_command = neutral_steering_command();
      send_steering_command(&steering_command);
      continue;
    }

    if (sbus_read_data(sbus_data) != APP_OK) {
      steering_command = neutral_steering_command();
      send_steering_command(&steering_command);
      continue;
    }

    // Parse packet
    sbus_parse_data(sbus_data, &sbus_packet);

    // The radio receiver sends a neutral command when the failsafe is activated
    // or a frame is lost, so we still want to send the data over CAN.

    // Failsafe usually triggers if many frames are lost in a row Indicates
    // connection loss (heavy). This will be handled by the radio receiver,
    // so we do nothing.
    if (sbus_packet.failsafe_activated) {
      print("Failsafe activated\r\n");
    }

    // Indicates slight connection loss or issue with frame.
    // Also handled by the receiver.
    if (sbus_packet.frame_lost) {
      print("Frame lost\r\n");
    }

    steering_command = sbus_packet_to_steering_command(&sbus_packet);
    send_steering_command(&steering_command);
  }
}

// Try to read a header in a loop. Return error on timeout or bad UART read.
int sbus_read_header(uint8_t *sbus_data) {
  peripherals_t *peripherals = get_peripherals();
  const uint16_t sbus_timeout_ms = 10;
  uint32_t uart_error = HAL_UART_ERROR_NONE;

  // Wait until reception of a header, in case we power up in the
  // middle of a transmission
  while (sbus_data[0] != SBUS_HEADER) {
    int status = APP_OK;

    HAL_UART_Receive_IT(&peripherals->huart2, sbus_data, sizeof(uint8_t));

    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(sbus_timeout_ms)) != pdPASS) {
      // Timeout
      status = APP_NOT_OK;
    }

    // Error check
    uart_error = HAL_UART_GetError(&peripherals->huart2);
    if (uart_error != HAL_UART_ERROR_NONE) {
      char str[32];  // NOLINT
      sprintf(str, "UART error in SBUS header: %u\r\n", uart_error);
      print(str);
      // Handle overrun by clearing receive register
      if (uart_error == HAL_UART_ERROR_ORE) {
        __HAL_UART_FLUSH_DRREGISTER(&peripherals->huart2);
      }
      status = APP_NOT_OK;
    }

    if (status != APP_OK) {
      return status;
    }
  }

  return APP_OK;
}

// Try to read an sbus packet.
int sbus_read_data(uint8_t *sbus_data) {
  peripherals_t *peripherals = get_peripherals();
  uint32_t uart_error = HAL_UART_ERROR_NONE;
  const uint16_t sbus_timeout_ms = 10;

  int status = APP_OK;

  // If header has been received correctly, let's receive the rest of the
  // packet.
  HAL_UART_Receive_IT(&peripherals->huart2, &sbus_data[1],
                      SBUS_PACKET_LENGTH - 1);

  // Don't wait forever, since radio connection could be lost mid transmission
  if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(sbus_timeout_ms)) != pdPASS) {
    // Timeout
    status = APP_NOT_OK;
  }

  // Check for uart errors
  uart_error = HAL_UART_GetError(&peripherals->huart2);
  if (uart_error != HAL_UART_ERROR_NONE) {
    char str[32];  // NOLINT
    sprintf(str, "UART error in SBUS data: %u\r\n", uart_error);
    print(str);
    // Handle overrun by clearing receive register
    if (uart_error == HAL_UART_ERROR_ORE) {
      __HAL_UART_FLUSH_DRREGISTER(&peripherals->huart2);
    }
    status = APP_NOT_OK;
  }

  return status;
}

void send_steering_command(steering_command_t *command) {
  ck_data_t *ck_data = get_ck_data();

  memcpy(&ck_data->steering_page->lines[1], &command->steering,
         sizeof(command->steering));
  memcpy(&ck_data->steering_trim_page->lines[1], &command->steering_trim,
         sizeof(command->steering_trim));
  memcpy(&ck_data->throttle_page->lines[1], &command->throttle,
         sizeof(command->throttle));
  memcpy(&ck_data->throttle_trim_page->lines[1], &command->throttle_trim,
         sizeof(command->throttle_trim));

  if (ck_send_document(ck_data->steering_folder->folder_no) != CK_OK) {
    print("failed to send doc.\r\n");
  }
  if (ck_send_document(ck_data->steering_trim_folder->folder_no) != CK_OK) {
    print("failed to send doc.\r\n");
  }
  if (ck_send_document(ck_data->throttle_folder->folder_no) != CK_OK) {
    print("failed to send doc.\r\n");
  }
  if (ck_send_document(ck_data->throttle_trim_folder->folder_no) != CK_OK) {
    print("failed to send doc.\r\n");
  }
}

void process_letter(void *unused) {
  (void)unused;

  peripherals_t *peripherals = get_peripherals();

  CAN_RxHeaderTypeDef header;
  uint8_t data[CK_CAN_MAX_DLC];
  ck_letter_t letter;

  for (;;) {
    if (HAL_CAN_ActivateNotification(&peripherals->common_peripherals->hcan,
                                     CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
      print("Error activating interrupt.\r\n");
      error();
    }
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Process all messages
    while (HAL_CAN_GetRxMessage(&peripherals->common_peripherals->hcan,
                                CAN_RX_FIFO0, &header, data) == HAL_OK) {
      if (ck_correct_letter_received() != CK_OK) {
        print("CAN Kingdom error in ck_correct_letter_received().\r\n");
      }
      letter = frame_to_letter(&header, data);
      dispatch_letter(&letter);
    }
  }
}

void dispatch_letter(ck_letter_t *letter) {
  // Check for default letter
  if (ck_is_default_letter(letter) == CK_OK) {
    if (ck_default_letter_received() != CK_OK) {
      print("CAN Kingdom error in ck_default_letter_received().\r\n");
    }
  }
  // Check for king's letter
  else if (ck_is_kings_envelope(&letter->envelope) == CK_OK) {
    if (ck_process_kings_letter(letter) != CK_OK) {
      print("failed to process king's letter.\r\n");
    }
  }
  // Check for any other letter
  // TODO
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance != USART2) {
    return;
  }
  BaseType_t higher_priority_task_woken = pdFALSE;
  vTaskNotifyGiveFromISR(sbus_read_task, &higher_priority_task_woken);
  portYIELD_FROM_ISR(higher_priority_task_woken);
}

// Deactivate interrupt, then signal task. Let the task reactivate the
// interrupt.
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  HAL_CAN_DeactivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

  BaseType_t higher_priority_task_woken = pdFALSE;
  vTaskNotifyGiveFromISR(process_letter_task, &higher_priority_task_woken);
  portYIELD_FROM_ISR(higher_priority_task_woken);
}
