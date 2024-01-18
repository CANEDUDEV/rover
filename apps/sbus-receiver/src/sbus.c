#include "sbus.h"

#include <string.h>

/*
 * The SBUS packet is 25 bytes long consisting of:
 *
 * Byte[0]: SBUS header, 0x0F
 * Byte[1-22]: 16 servo channels, 11 bits each
 * Byte[23]
 *     Bit 0: channel 17 (0x01)
 *     Bit 1: channel 18 (0x02)
 *     Bit 2: frame lost (0x04)
 *     Bit 3: failsafe activated (0x08)
 * Byte[24]: SBUS footer
 *
 */
void sbus_parse_data(const uint8_t *data, sbus_packet_t *sbus_packet) {
  memset(sbus_packet, 0, sizeof(sbus_packet_t));

  // NOLINTBEGIN(*-magic-numbers)
  sbus_packet->channels[0] = data[1] | ((data[2] << 8) & 0x07FF);
  sbus_packet->channels[1] = (data[2] >> 3) | ((data[3] << 5) & 0x07FF);
  sbus_packet->channels[2] =
      (data[3] >> 6) | (data[4] << 2) | ((data[5] << 10) & 0x07FF);

  sbus_packet->channels[3] = (data[5] >> 1) | ((data[6] << 7) & 0x07FF);
  sbus_packet->channels[4] = (data[6] >> 4) | ((data[7] << 4) & 0x07FF);
  sbus_packet->channels[5] =
      (data[7] >> 7) | (data[8] << 1) | ((data[9] << 9) & 0x07FF);

  sbus_packet->channels[6] = (data[9] >> 2) | ((data[10] << 6) & 0x07FF);
  sbus_packet->channels[7] = (data[10] >> 5) | ((data[11] << 3) & 0x07FF);
  sbus_packet->channels[8] = data[12] | ((data[13] << 8) & 0x07FF);
  sbus_packet->channels[9] = (data[13] >> 3) | ((data[14] << 5) & 0x07FF);
  sbus_packet->channels[10] =
      (data[14] >> 6) | (data[15] << 2) | ((data[16] << 10) & 0x07FF);

  sbus_packet->channels[11] = (data[16] >> 1) | ((data[17] << 7) & 0x07FF);
  sbus_packet->channels[12] = (data[17] >> 4) | ((data[18] << 4) & 0x07FF);
  sbus_packet->channels[13] =
      (data[18] >> 7) | (data[19] << 1) | ((data[20] << 9) & 0x07FF);

  sbus_packet->channels[14] = (data[20] >> 2) | ((data[21] << 6) & 0x07FF);
  sbus_packet->channels[15] = (data[21] >> 5) | ((data[22] << 3) & 0x07FF);

  sbus_packet->channels[16] = data[23] & 0x01;
  sbus_packet->channels[17] = data[23] & 0x02;
  sbus_packet->frame_lost = data[23] & 0x04;
  sbus_packet->failsafe_activated = data[23] & 0x08;
  // NOLINTEND(*-magic-numbers)
}
