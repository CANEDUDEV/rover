#include "postmaster-hal.h"

#include <string.h>

static CAN_HandleTypeDef *hcan;

void postmaster_init(CAN_HandleTypeDef *_hcan) { hcan = _hcan; }

CAN_HandleTypeDef *get_can_handle(void) { return hcan; }

ck_letter_t frame_to_letter(CAN_RxHeaderTypeDef *header, uint8_t *data) {
  ck_letter_t letter;
  letter.envelope.is_remote = header->RTR;
  letter.envelope.has_extended_id = header->IDE;
  if (letter.envelope.has_extended_id) {
    letter.envelope.envelope_no = header->ExtId;
  } else {
    letter.envelope.envelope_no = header->StdId;
  }
  letter.envelope.is_compressed = false;
  letter.page.line_count = header->DLC;
  memcpy(letter.page.lines, data, header->DLC);
  return letter;
}
