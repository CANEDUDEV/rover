#ifndef CAN_UTILS_H
#define CAN_UTILS_H

#include <stdint.h>

#define CAN_MAX_DLC 8
#define CAN_MESSAGE_QUEUE_LENGTH 10

typedef struct {
  uint32_t id;
  uint8_t dlc;
  uint8_t data[CAN_MAX_DLC];
} CANFrame;

// 'argument' parameter should be a pointer to a QueueHandle_t
// that stores items of type 'CANFrame'
void StartCANTxTask(void *argument);

#endif /* CAN_UTILS_H */
