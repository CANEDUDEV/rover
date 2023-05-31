#include "types.h"

#include "postmaster.h"

inline int min(int a, int b) {  // NOLINT
  if (a < b) {
    return a;
  }
  return b;
}

ck_err_t ck_check_action_mode(ck_action_mode_t mode) {
  switch (mode) {
    case CK_ACTION_MODE_KEEP_CURRENT:
    case CK_ACTION_MODE_RUN:
    case CK_ACTION_MODE_FREEZE:
    case CK_ACTION_MODE_RESET:
      return CK_OK;
    default:
      return CK_ERR_INVALID_ACTION_MODE;
  }
}

ck_err_t ck_check_comm_mode(ck_comm_mode_t mode) {
  switch (mode) {
    case CK_COMM_MODE_KEEP_CURRENT:
    case CK_COMM_MODE_SILENT:
    case CK_COMM_MODE_LISTEN_ONLY:
    case CK_COMM_MODE_COMMUNICATE:
      return CK_OK;
    default:
      return CK_ERR_INVALID_COMM_MODE;
  }
}

ck_err_t ck_check_list_type(ck_list_type_t type) {
  switch (type) {
    case CK_LIST_BIT:
    case CK_LIST_LINE:
    case CK_LIST_PAGE:
    case CK_LIST_DOCUMENT:
      return CK_OK;
    default:
      return CK_ERR_INVALID_LIST_TYPE;
  }
}

ck_err_t ck_check_can_bit_timing(const ck_can_bit_timing_t *bit_timing) {
  if (!bit_timing) {
    return CK_ERR_INVALID_CAN_ID;
  }
  // NOLINTBEGIN(*-magic-numbers)
  int tseg1 = bit_timing->time_quanta - 1 - bit_timing->phase_seg2;
  if (bit_timing->prescaler < 1 || bit_timing->prescaler > 32) {
    return CK_ERR_INVALID_CAN_BIT_TIMING;
  }
  if (bit_timing->time_quanta < 8 || bit_timing->time_quanta > 25) {
    return CK_ERR_INVALID_CAN_BIT_TIMING;
  }
  if (bit_timing->phase_seg2 < 1 || bit_timing->phase_seg2 > 8) {
    return CK_ERR_INVALID_CAN_BIT_TIMING;
  }
  if (tseg1 < 1) {
    return CK_ERR_INVALID_CAN_BIT_TIMING;
  }
  if (bit_timing->sjw < 1 || bit_timing->sjw > 4 ||
      bit_timing->sjw > min(tseg1, bit_timing->phase_seg2)) {
    return CK_ERR_INVALID_CAN_BIT_TIMING;
  }
  // NOLINTEND(*-magic-numbers)
  return CK_OK;
}

ck_letter_t ck_default_letter(void) {
  ck_letter_t letter = {
      .envelope =
          {
              .envelope_no = CK_DEFAULT_LETTER_ENVELOPE,
          },
      .page =
          {
              .line_count = CK_MAX_LINES_PER_PAGE,
              // NOLINTNEXTLINE(*-magic-numbers)
              .lines = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA},
          },
  };
  return letter;
}

ck_can_bit_timing_t ck_default_bit_timing(void) {
  ck_can_bit_timing_t bit_timing = {
      .prescaler = ck_get_125kbit_prescaler(),
      .time_quanta = 16,  // NOLINT
      .phase_seg2 = 2,
      .sjw = 1,
  };
  return bit_timing;
}
