#ifndef APP_H
#define APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void ReadSwitches(uint8_t *data);
void SendAnalogPortMessage(const uint16_t *data);
void SendSwitchPortMessage(const uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif /*APP_H */
