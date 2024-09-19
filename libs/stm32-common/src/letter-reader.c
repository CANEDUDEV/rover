#include "letter-reader.h"

#include <stdio.h>
#include <string.h>

#include "common-peripherals.h"
#include "error.h"
#include "stm32f3xx_hal_can.h"

// CK
#include "mayor.h"
#include "ck-types.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#define LETTER_QUEUE_LENGTH 10
#define LETTER_QUEUE_ITEM_SIZE sizeof(ck_letter_t)

static QueueHandle_t letter_queue;
static StaticQueue_t static_letter_queue;
static uint8_t
    letter_queue_storage[LETTER_QUEUE_LENGTH * LETTER_QUEUE_ITEM_SIZE];

static StaticTask_t process_letter_buf;

static StackType_t process_letter_stack[configMINIMAL_STACK_SIZE];

static void process_letter(void *unused);
static void dispatch_letter(ck_letter_t *letter);
static ck_letter_t frame_to_letter(CAN_RxHeaderTypeDef *header, uint8_t *data);

static letter_reader_cfg_t task_cfg;

int init_letter_reader_task(letter_reader_cfg_t config) {
  if (config.priority == 0) {
    return APP_NOT_OK;
  }
  if (config.app_letter_handler_func == NULL) {
    return APP_NOT_OK;
  }

  letter_queue = xQueueCreateStatic(LETTER_QUEUE_LENGTH, LETTER_QUEUE_ITEM_SIZE,
                                    letter_queue_storage, &static_letter_queue);

  xTaskCreateStatic(process_letter, "process letter", configMINIMAL_STACK_SIZE,
                    NULL, config.priority, process_letter_stack,
                    &process_letter_buf);

  task_cfg = config;

  return APP_OK;
}

static void process_letter(void *unused) {
  (void)unused;

  common_peripherals_t *common_peripherals = get_common_peripherals();

  ck_letter_t letter;

  if (HAL_CAN_ActivateNotification(&common_peripherals->hcan,
                                   CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
    printf("Error activating interrupt.\r\n");
    error();
  }

  for (;;) {
    if (xQueueReceive(letter_queue, &letter, portMAX_DELAY) != pdPASS) {
      printf("Error retrieving letter.\r\n");
      continue;
    }

    if (ck_correct_letter_received() != CK_OK) {
      printf("CAN Kingdom error in ck_correct_letter_received().\r\n");
    }

    dispatch_letter(&letter);
  }
}

static void dispatch_letter(ck_letter_t *letter) {
  ck_folder_t *folder = NULL;

  // Check for default letter
  if (ck_is_default_letter(letter) == CK_OK) {
    if (ck_default_letter_received() != CK_OK) {
      printf("CAN Kingdom error in ck_default_letter_received().\r\n");
    }
  }
  // Check for king's letter
  else if (ck_is_kings_envelope(&letter->envelope) == CK_OK) {
    if (ck_process_kings_letter(letter) != CK_OK) {
      printf("failed to process king's letter.\r\n");
    }
  }
  // Check for any other letter
  else if (ck_get_envelopes_folder(&letter->envelope, &folder) == CK_OK) {
    if (task_cfg.app_letter_handler_func(folder, letter) != APP_OK) {
      printf("failed to process page.\r\n");
    }
  }
}

static ck_letter_t frame_to_letter(CAN_RxHeaderTypeDef *header, uint8_t *data) {
  ck_letter_t letter;
  letter.envelope.is_remote = header->RTR;
  letter.envelope.has_extended_id = header->IDE;
  if (letter.envelope.has_extended_id) {
    letter.envelope.envelope_no = header->ExtId;
  } else {
    letter.envelope.envelope_no = header->StdId;
  }
  letter.envelope.is_compressed = false;
  letter.page.line_count = header->DLC;
  memcpy(letter.page.lines, data, header->DLC);
  return letter;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  CAN_RxHeaderTypeDef header;
  uint8_t data[CK_CAN_MAX_DLC];
  ck_letter_t letter;
  BaseType_t higher_priority_task_woken = pdFALSE;

  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) == HAL_OK) {
    letter = frame_to_letter(&header, data);
    xQueueSendFromISR(letter_queue, &letter, &higher_priority_task_woken);
  }

  portYIELD_FROM_ISR(higher_priority_task_woken);
}
