#ifndef APP_H
#define APP_H

#include <stdint.h>

void Print(char *str);
void ReadSwitches(uint8_t *data);
void SendAnalogPortMessage(const uint16_t *data);
void SendSwitchPortMessage(const uint8_t *data);

#endif /*APP_H */
