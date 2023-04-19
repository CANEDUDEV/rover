#ifndef UTILS_H
#define UTILS_H

#include "cmsis_os.h"

#define CAN_MAX_DLC 8

typedef struct {
  uint32_t id;
  uint8_t dlc;
  uint8_t data[CAN_MAX_DLC];
} CANFrame;

void Print(char *str);
void SignalTask(osThreadId_t taskHandle);

#endif
