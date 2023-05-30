#include "king.h"

#include <string.h>

// We are addressing individual bytes when constructing the king's pages.
// NOLINTBEGIN(*magic-numbers)

ck_err_t ck_create_kings_page_0(const ck_kp0_args_t *args, ck_page_t *page) {
  // Null pointer check
  if (!args || !page) {
    return CK_ERR_INVALID_PARAMETER;
  }
  ck_err_t ret = ck_check_action_mode(args->action_mode);
  if (ret != CK_OK) {
    return ret;
  }
  ret = ck_check_comm_mode(args->comm_mode);
  if (ret != CK_OK) {
    return ret;
  }
  page->line_count = CK_MAX_LINES_PER_PAGE;
  page->lines[0] = args->address;
  page->lines[1] = CK_KP0;
  page->lines[2] = args->action_mode;
  page->lines[3] = args->comm_mode | args->comm_flags;
  page->lines[4] = args->city_mode;
  page->lines[5] = 0;
  page->lines[6] = 0;
  page->lines[7] = 0;
  return CK_OK;
}

ck_err_t ck_create_kings_page_1(const ck_kp1_args_t *args, ck_page_t *page) {
  // Null pointer check
  if (!args || !page) {
    return CK_ERR_INVALID_PARAMETER;
  }
  // CAN ID bounds check
  if ((!args->has_extended_id && args->base_no > CK_CAN_MAX_STD_ID) ||
      (args->has_extended_id && args->base_no > CK_CAN_MAX_EXT_ID)) {
    return CK_ERR_INVALID_CAN_ID;
  }

  page->line_count = CK_MAX_LINES_PER_PAGE;
  page->lines[0] = args->address;
  page->lines[1] = CK_KP1;
  page->lines[2] = args->mayor_response_no;
  page->lines[3] = 0;

  memcpy(&page->lines[4], &args->base_no, sizeof(args->base_no));

  page->lines[7] |= (args->has_extended_id << 7);

  return CK_OK;
}

ck_err_t ck_create_kings_page_2(const ck_kp2_args_t *args, ck_page_t *page) {
  // Null pointer check
  if (!args || !page) {
    return CK_ERR_INVALID_PARAMETER;
  }
  // CAN ID bounds check
  if ((!args->envelope.has_extended_id &&
       args->envelope.envelope_no > CK_CAN_MAX_STD_ID) ||
      (args->envelope.has_extended_id &&
       args->envelope.envelope_no > CK_CAN_MAX_EXT_ID)) {
    return CK_ERR_INVALID_CAN_ID;
  }

  // Not allowed to enable an envelope that is not assigned
  if (args->envelope.enable && args->envelope_action == CK_ENVELOPE_EXPEL) {
    return CK_ERR_INCOMPATIBLE_PARAMS;
  }

  page->line_count = CK_MAX_LINES_PER_PAGE;
  page->lines[0] = args->address;
  page->lines[1] = CK_KP2;

  memcpy(&page->lines[2], &args->envelope.envelope_no,
         sizeof(args->envelope.envelope_no));

  page->lines[5] |= (args->envelope.is_compressed << 6);
  page->lines[5] |= (args->envelope.has_extended_id << 7);

  page->lines[6] = args->folder_no;

  page->lines[7] = (args->envelope.enable) | (args->envelope_action << 1);

  return CK_OK;
}

ck_err_t ck_create_kings_page_8(uint8_t address,
                                const ck_can_bit_timing_t *bit_timing,
                                ck_page_t *page) {
  // Null pointer check
  if (!bit_timing | !page) {
    return CK_ERR_INVALID_PARAMETER;
  }

  page->line_count = CK_MAX_LINES_PER_PAGE;
  page->lines[0] = address;
  page->lines[1] = CK_KP8;
  page->lines[2] = 0;
  page->lines[3] = 0;
  page->lines[4] = bit_timing->prescaler;
  page->lines[5] = bit_timing->time_quanta;
  page->lines[6] = bit_timing->phase_seg2;
  page->lines[7] = bit_timing->sjw;

  return CK_OK;
}

ck_err_t ck_create_kings_page_16(const ck_kp16_args_t *args, ck_page_t *page) {
  // Null pointer check
  if (!args || !page) {
    return CK_ERR_INVALID_PARAMETER;
  }
  // Folders 0 and 1 are reserved for the king's document and the mayor's
  // document, respectively.
  if (args->folder_no < 2) {
    return CK_ERR_INVALID_FOLDER_NUMBER;
  }
  // DLC bounds check.
  if (args->dlc > CK_CAN_MAX_DLC) {
    return CK_ERR_INVALID_CAN_DLC;
  }

  page->line_count = CK_MAX_LINES_PER_PAGE;
  page->lines[0] = args->address;
  page->lines[1] = CK_KP16;
  page->lines[2] = args->folder_no;

  // MSB is set to 1 in CK specification
  page->lines[3] = (1 << 7) | (args->has_rtr << 6) | args->dlc;

  // If all bits except the MSB are 0, then settings should be unchanged and MSB
  // set to 0.
  if (page->lines[3] == (1 << 7)) {
    page->lines[3] = 0;
  }

  // MSB is set to 1 in CK specification
  page->lines[4] = (1 << 7) | (args->enable_folder << 6) |
                   (args->document_action << 4) | args->direction;

  // If all bits except the MSB are 0, then settings should be unchanged and MSB
  // set to 0.
  if (page->lines[4] == (1 << 7)) {
    page->lines[4] = 0;
  }

  page->lines[5] = args->list_no;
  page->lines[6] = args->document_no;
  page->lines[7] = 0;

  return CK_OK;
}

ck_err_t ck_create_kings_page_17(const ck_kp17_args_t *args, ck_page_t *page) {
  // Null pointer check
  if (!args || !page) {
    return CK_ERR_INVALID_PARAMETER;
  }
  ck_err_t ret = ck_check_list_type(args->list_type);
  if (ret != CK_OK) {
    return ret;
  }
  page->line_count = CK_MAX_LINES_PER_PAGE;
  page->lines[0] = args->address;
  page->lines[1] = CK_KP17;
  page->lines[2] = (args->list_type << 3) | args->direction;
  page->lines[3] = args->source_list_no;
  page->lines[4] = args->source_record_no;
  page->lines[5] = args->target_list_no;
  page->lines[6] = args->target_record_no;
  page->lines[7] = args->position;
  return CK_OK;
}

// NOLINTEND(*magic-numbers)
