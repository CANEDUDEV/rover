#include "can-utils.h"

#include "cmsis_os.h"
#include "main.h"

// FreeRTOS includes
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

// Common utils
#include "utils.h"

// Defined in main.c
extern CAN_HandleTypeDef hcan;
extern osThreadId_t CANTxTaskHandle;

void mailboxFreeCallback(CAN_HandleTypeDef *_hcan) {
  // Only signal when going from 0 free mailboxes to 1 free
  if (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) <= 1) {
    SignalTask(CANTxTaskHandle);
  }
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *_hcan) {
  mailboxFreeCallback(_hcan);
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *_hcan) {
  mailboxFreeCallback(_hcan);
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *_hcan) {
  mailboxFreeCallback(_hcan);
}

void StartCANTxTask(void *argument) {
  QueueHandle_t *CANMessageQueue = (QueueHandle_t *)argument;

  HAL_CAN_ActivateNotification(&hcan, CAN_IT_TX_MAILBOX_EMPTY);
  HAL_CAN_Start(&hcan);

  uint32_t mailbox = 0;

  for (;;) {
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0) {
      // Wait for mailbox to become available
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }

    CANFrame frame;
    // Wait for queue to receive message
    xQueueReceive(*CANMessageQueue, &frame, portMAX_DELAY);

    CAN_TxHeaderTypeDef header = {
        .StdId = frame.id,
        .DLC = frame.dlc,
    };

    HAL_CAN_AddTxMessage(&hcan, &header, frame.data, &mailbox);
  }
}
