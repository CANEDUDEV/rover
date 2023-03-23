#ifndef UTILS_H
#define UTILS_H

#include "cmsis_os.h"

void Print(char *str);
void SignalTask(osThreadId_t taskHandle);

#endif
