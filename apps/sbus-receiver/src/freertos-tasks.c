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

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"

#define LOWEST_TASK_PRIORITY 24

static TaskHandle_t sbus_read_task;
static StaticTask_t sbus_read_buffer;
static StackType_t sbus_read_stack[configMINIMAL_STACK_SIZE];

// CAN Kingdom process received letters task
static TaskHandle_t process_letter_task;
static StaticTask_t process_letter_buf;
static StackType_t process_letter_stack[configMINIMAL_STACK_SIZE];

void sbus_read(void *unused);
void process_letter(void *unused);

void send_docs(void);
void dispatch_letter(ck_letter_t *letter);

void task_init(void) {
  sbus_read_task = xTaskCreateStatic(
      sbus_read, "sbus read", configMINIMAL_STACK_SIZE, NULL,
      LOWEST_TASK_PRIORITY, sbus_read_stack, &sbus_read_buffer);

  process_letter_task = xTaskCreateStatic(
      process_letter, "process letter", configMINIMAL_STACK_SIZE, NULL,
      LOWEST_TASK_PRIORITY + 1, process_letter_stack, &process_letter_buf);
}

void sbus_read(void *unused) {
  (void)unused;

  peripherals_t *peripherals = get_peripherals();

  ck_data_t *ck_data = get_ck_data();

  uint8_t sbus_data[SBUS_PACKET_LENGTH];
  sbus_packet_t sbus_packet;
  steering_command_t steering_command;

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

    memcpy(&ck_data->steering_page->lines[1], &steering_command.steering,
           sizeof(steering_command.steering));
    memcpy(&ck_data->steering_trim_page->lines[1],
           &steering_command.steering_trim,
           sizeof(steering_command.steering_trim));
    memcpy(&ck_data->throttle_page->lines[1], &steering_command.throttle,
           sizeof(steering_command.throttle));
    memcpy(&ck_data->throttle_trim_page->lines[1],
           &steering_command.throttle_trim,
           sizeof(steering_command.throttle_trim));

    send_docs();
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

void send_docs(void) {
  ck_data_t *ck_data = get_ck_data();

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
  (void)huart;
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
