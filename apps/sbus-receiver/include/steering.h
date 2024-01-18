#ifndef STEERING_H
#define STEERING_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "sbus.h"

typedef struct {
  int16_t steering_angle;
  int16_t throttle;
} steering_command_t;

steering_command_t sbus_packet_to_steering_command(
    const sbus_packet_t *sbus_packet);

steering_command_t neutral_steering_command(void);

#ifdef __cplusplus
}
#endif

#endif /* STEERING_H */
