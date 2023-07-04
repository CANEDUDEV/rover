#ifndef SBUS_H
#define SBUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define SBUS_HEADER 0x0F
#define SBUS_CHANNEL_COUNT 18
#define SBUS_PACKET_LENGTH 25

typedef struct {
  uint16_t channels[SBUS_CHANNEL_COUNT];
  bool frame_lost;
  bool failsafe_activated;
} sbus_packet_t;

void sbus_parse_data(const uint8_t *data, sbus_packet_t *sbus_packet);

#ifdef __cplusplus
}
#endif

#endif /* SBUS_H */
