#ifndef SBUS_H
#define SBUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define SBUS_CHANNEL_COUNT 18

typedef struct {
  uint16_t channels[SBUS_CHANNEL_COUNT];
  bool frame_lost;
  bool failsafe_activated;
} sbus_message_t;

int sbus_read_message(sbus_message_t *message);

#ifdef __cplusplus
}
#endif

#endif /* SBUS_H */
