#include "postmaster.h"

#include "error.h"
#include "flash.h"
#include "postmaster-hal.h"

// Helpers
static ck_err_t check_bit_timing(const ck_can_bit_timing_t *bit_timing);
static int get_tseg1(const ck_can_bit_timing_t *bit_timing);
static int get_tseg2(const ck_can_bit_timing_t *bit_timing);
static int get_sjw(const ck_can_bit_timing_t *bit_timing);

inline int min(int a, int b) {  // NOLINT
  if (a < b) {
    return a;
  }
  return b;
}

ck_err_t ck_send_letter(const ck_letter_t *letter, uint8_t dlc) {
  CAN_HandleTypeDef *hcan = get_can_handle();

  // If bus off, return error
  if (hcan->State != HAL_CAN_STATE_READY &&
      hcan->State != HAL_CAN_STATE_LISTENING) {
    return CK_ERR_SEND_FAILED;
  }

  while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) < 1) {
    // Busy loop, wait for mailbox to be free
  }

  CAN_TxHeaderTypeDef header = {
      .DLC = dlc,
      .RTR = CAN_RTR_DATA,
      .IDE = CAN_ID_STD,
      .StdId = letter->envelope.envelope_no,
  };

  if (letter->envelope.is_remote) {
    header.RTR = CAN_RTR_REMOTE;
  }
  if (letter->envelope.has_extended_id) {
    header.IDE = CAN_ID_EXT;
    header.ExtId = letter->envelope.envelope_no;
  }

  uint32_t mailbox = 0;
  if (HAL_CAN_AddTxMessage(hcan, &header, letter->page.lines, &mailbox) !=
      HAL_OK) {
    return CK_ERR_SEND_FAILED;
  }
  return CK_OK;
}

ck_err_t ck_apply_comm_mode(ck_comm_mode_t mode) {
  ck_err_t ret = ck_check_comm_mode(mode);
  if (ret != CK_OK) {
    return ret;
  }

  CAN_HandleTypeDef *hcan = get_can_handle();
  switch (mode) {
    case CK_COMM_MODE_COMMUNICATE:
    case CK_COMM_MODE_LISTEN_ONLY:
      if (HAL_CAN_GetState(hcan) == HAL_CAN_STATE_LISTENING) {
        if (HAL_CAN_Stop(hcan) != HAL_OK) {
          return CK_ERR_SET_MODE_FAILED;
        }
      }

      hcan->Init.Mode = CAN_MODE_NORMAL;
      if (HAL_CAN_Init(hcan) != HAL_OK) {
        return CK_ERR_SET_MODE_FAILED;
      }

      if (HAL_CAN_Start(hcan) != HAL_OK) {
        return CK_ERR_SET_MODE_FAILED;
      }
      break;

    case CK_COMM_MODE_SILENT:
      if (HAL_CAN_GetState(hcan) == HAL_CAN_STATE_LISTENING) {
        if (HAL_CAN_Stop(hcan) != HAL_OK) {
          return CK_ERR_SET_MODE_FAILED;
        }
      }

      hcan->Init.Mode = CAN_MODE_SILENT;
      if (HAL_CAN_Init(hcan) != HAL_OK) {
        return CK_ERR_SET_MODE_FAILED;
      }

      if (HAL_CAN_Start(hcan) != HAL_OK) {
        return CK_ERR_SET_MODE_FAILED;
      }
      break;

    default:
      break;
  }
  return CK_OK;
}

uint8_t ck_get_125kbit_prescaler(void) {
  return 18;  // NOLINT
}

ck_err_t ck_set_bit_timing(const ck_can_bit_timing_t *bit_timing) {
  if (!bit_timing) {
    return CK_ERR_INVALID_PARAMETER;
  }

  ck_err_t ret = check_bit_timing(bit_timing);
  if (ret != CK_OK) {
    return ret;
  }

  CAN_HandleTypeDef *hcan = get_can_handle();

  // Store old values
  CAN_InitTypeDef old_init = hcan->Init;

  hcan->Init.Prescaler = bit_timing->prescaler;
  hcan->Init.TimeSeg1 = get_tseg1(bit_timing);
  hcan->Init.TimeSeg2 = get_tseg2(bit_timing);
  hcan->Init.SyncJumpWidth = get_sjw(bit_timing);

  // Go off bus
  if (HAL_CAN_GetState(hcan) == HAL_CAN_STATE_LISTENING) {
    if (HAL_CAN_Stop(hcan) != HAL_OK) {
      return CK_ERR_PERIPHERAL;
    }
  }

  if (HAL_CAN_Init(hcan) != HAL_OK) {
    // Try to recover bit timing
    hcan->Init = old_init;
    HAL_CAN_Init(hcan);
    return CK_ERR_INVALID_CAN_BIT_TIMING;
  }

  return CK_OK;
}

ck_err_t ck_save_bit_timing(const ck_can_bit_timing_t *bit_timing) {
  if (flash_write(FLASH_RW_START, bit_timing, sizeof(ck_can_bit_timing_t)) !=
      APP_OK) {
    return CK_ERR_PERIPHERAL;
  }
  return CK_OK;
}

ck_err_t ck_load_bit_timing(ck_can_bit_timing_t *bit_timing) {
  if (flash_read(FLASH_RW_START, bit_timing, sizeof(ck_can_bit_timing_t)) !=
      APP_OK) {
    return CK_ERR_PERIPHERAL;
  }
  return CK_OK;
}

// Check bit timing based on stm32 hardware constraints.
static ck_err_t check_bit_timing(const ck_can_bit_timing_t *bit_timing) {
  if (bit_timing->prescaler < 1) {
    return CK_ERR_INVALID_CAN_BIT_TIMING;
  }
  if (bit_timing->phase_seg2 < 1 || bit_timing->phase_seg2 > 8) {  // NOLINT
    return CK_ERR_INVALID_CAN_BIT_TIMING;
  }
  int tseg1 = bit_timing->time_quanta - 1 - bit_timing->phase_seg2;
  if (tseg1 < 1 || tseg1 > 16) {  // NOLINT
    return CK_ERR_INVALID_CAN_BIT_TIMING;
  }
  if (bit_timing->sjw < 1 || bit_timing->sjw > 4) {
    return CK_ERR_INVALID_CAN_BIT_TIMING;
  }
  if (bit_timing->sjw > min(tseg1, bit_timing->phase_seg2)) {
    return CK_ERR_INVALID_CAN_BIT_TIMING;
  }
  return CK_OK;
}

static int get_tseg1(const ck_can_bit_timing_t *bit_timing) {
  int tseg1 = bit_timing->time_quanta - 1 - bit_timing->phase_seg2;

  // NOLINTBEGIN(*-magic-numbers)
  switch (tseg1) {
    case 1:
      return CAN_BS1_1TQ;
    case 2:
      return CAN_BS1_2TQ;
    case 3:
      return CAN_BS1_3TQ;
    case 4:
      return CAN_BS1_4TQ;
    case 5:
      return CAN_BS1_5TQ;
    case 6:
      return CAN_BS1_6TQ;
    case 7:
      return CAN_BS1_7TQ;
    case 8:
      return CAN_BS1_8TQ;
    case 9:
      return CAN_BS1_9TQ;
    case 10:
      return CAN_BS1_10TQ;
    case 11:
      return CAN_BS1_11TQ;
    case 12:
      return CAN_BS1_12TQ;
    case 13:
      return CAN_BS1_13TQ;
    case 14:
      return CAN_BS1_14TQ;
    case 15:
      return CAN_BS1_15TQ;
    case 16:
      return CAN_BS1_16TQ;
    default:
      return tseg1;
  }
  // NOLINTEND(*-magic-numbers)
}

static int get_tseg2(const ck_can_bit_timing_t *bit_timing) {
  // NOLINTBEGIN(*-magic-numbers)
  switch (bit_timing->phase_seg2) {
    case 1:
      return CAN_BS2_1TQ;
    case 2:
      return CAN_BS2_2TQ;
    case 3:
      return CAN_BS2_3TQ;
    case 4:
      return CAN_BS2_4TQ;
    case 5:
      return CAN_BS2_5TQ;
    case 6:
      return CAN_BS2_6TQ;
    case 7:
      return CAN_BS2_7TQ;
    case 8:
      return CAN_BS2_8TQ;
    default:
      return bit_timing->phase_seg2;
  }
  // NOLINTEND(*-magic-numbers)
}

static int get_sjw(const ck_can_bit_timing_t *bit_timing) {
  switch (bit_timing->sjw) {
    case 1:
      return CAN_SJW_1TQ;
    case 2:
      return CAN_SJW_2TQ;
    case 3:
      return CAN_SJW_3TQ;
    case 4:
      return CAN_SJW_4TQ;
    default:
      return bit_timing->sjw;
  }
}
