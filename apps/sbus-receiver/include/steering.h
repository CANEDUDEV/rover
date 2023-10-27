#ifndef STEERING_H
#define STEERING_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "sbus.h"

// The values correspond to PWM pulses, i.e. 1000 is min and 2000 is max.
typedef struct {
  uint16_t steering;
  int16_t steering_trim;
  uint16_t throttle;
  int16_t throttle_trim;
} steering_command_t;

steering_command_t sbus_packet_to_steering_command(
    const sbus_packet_t *sbus_packet);

steering_command_t neutral_steering_command(void);

bool steering_commands_are_equal(const steering_command_t *cmd1,
                                 const steering_command_t *cmd2);

#ifdef __cplusplus
}
#endif

#endif /* STEERING_H */
