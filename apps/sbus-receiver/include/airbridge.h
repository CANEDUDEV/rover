#ifndef AIRBRIDGE_H
#define AIRBRIDGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "sbus.h"

#define AIRBRIDGE_DISCONNECT_ID 0x4FF

void handle_airbridge_command(sbus_message_t* message);
void send_airbridge_disconnect(void);

#ifdef __cplusplus
}
#endif

#endif /* AIRBRIDGE_H */
