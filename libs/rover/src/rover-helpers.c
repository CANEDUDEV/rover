#include "rover-helpers.h"

#include <stdio.h>

#include "rover-assignments.h"
#include "rover-defs.h"

// CK
#include "king.h"
#include "mayor.h"
#include "postmaster.h"

static ck_envelope_t kings_envelope = {
    .envelope_no = 0,
    .enable = true,
    .is_remote = false,
    .is_compressed = false,
    .has_extended_id = false,
};

ck_err_t send_default_letter(void) {
  // Spoof a default letter reception
  ck_err_t ret = ck_default_letter_received();
  if (ret != CK_OK) {
    printf("Error: failed to receive default letter: %d\r\n", ret);
    return ret;
  }

  ck_letter_t default_letter = ck_default_letter();
  ret = ck_send_letter(&default_letter);
  if (ret != CK_OK) {
    printf("Error: failed to send default letter: %d\r\n", ret);
    return ret;
  }

  return CK_OK;
}

ck_err_t assign_rover_envelopes(const ck_id_t *own_id) {
  init_rover_kingdom();
  rover_kingdom_t *kingdom = get_rover_kingdom();

  for (size_t i = 0; i < kingdom->assignment_count; i++) {
    ck_page_t page;
    ck_kp2_args_t kp2_args = {
        .address = kingdom->assignments[i].city,
        .envelope.enable = true,
        .envelope.envelope_no = kingdom->assignments[i].envelope,
        .folder_no = kingdom->assignments[i].folder,
        .envelope_action = CK_ENVELOPE_ASSIGN,
    };
    ck_err_t ret = ck_create_kings_page_2(&kp2_args, &page);
    if (ret != CK_OK) {
      printf("Error: failed to create king's page: %d.\r\n", ret);
      return ret;
    }

    ck_letter_t letter = {
        .envelope = kings_envelope,
        .page = page,
    };

    // Letters meant for us need to be processed internally,
    // since we can't receive the messages we send.
    if (kingdom->assignments[i].city == own_id->city_address) {
      ret = ck_process_kings_letter(&letter);
      if (ret != CK_OK) {
        printf("Error: failed to assign envelope to self: %d.\r\n", ret);
        return ret;
      }
    }

    ret = ck_send_letter(&letter);
    if (ret != CK_OK) {
      printf("Error: failed to assign envelope: %d\r\n", ret);
      return ret;
    }
  }

  return CK_OK;
}

ck_err_t set_rover_base_number(void) {
  ck_page_t page;
  ck_kp1_args_t kp1_args = {
      .address = 0,
      .base_no = ROVER_BASE_NUMBER,
      .has_extended_id = false,
      .mayor_response_no = 0,
  };
  ck_err_t ret = ck_create_kings_page_1(&kp1_args, &page);
  if (ret != CK_OK) {
    printf("Error: failed to create king's page: %d.\r\n", ret);
    return ret;
  }

  ck_letter_t letter = {
      .envelope = kings_envelope,
      .page = page,
  };

  // Send base number to self
  ret = ck_process_kings_letter(&letter);
  if (ret != CK_OK) {
    printf("Error: couldn't send base number letter to self: %d.\r\n", ret);
    return ret;
  }

  ret = ck_send_letter(&letter);
  if (ret != CK_OK) {
    printf("Error: couldn't send base number letter: %d.\r\n", ret);
    return ret;
  }

  return CK_OK;
}

// Only setting so far is the reversal of the servo.
ck_err_t configure_rover_settings(void) {
  ck_envelope_t reverse_envelope = {
      .envelope_no = ROVER_ENVELOPE_SERVO_REVERSE_DIRECTION,
      .enable = true,
      .has_extended_id = false,
      .is_compressed = false,
      .is_remote = false,
  };

  ck_page_t page = {.line_count = 0};

  ck_letter_t letter = {
      .page = page,
      .envelope = reverse_envelope,
  };

  ck_err_t ret = ck_send_letter(&letter);
  if (ret != CK_OK) {
    printf("Error: failed to send servo reverse letter: %d\r\n", ret);
    return ret;
  }

  return CK_OK;
}

ck_err_t start_communication(void) {
  ck_page_t page;
  ck_kp0_args_t kp0_args = {
      .address = 0,
      .city_mode = CK_CITY_MODE_KEEP_CURRENT,
      .action_mode = CK_ACTION_MODE_KEEP_CURRENT,
      .comm_mode = CK_COMM_MODE_COMMUNICATE,
  };
  ck_err_t ret = ck_create_kings_page_0(&kp0_args, &page);
  if (ret != CK_OK) {
    printf("Error: failed to create king's page: %d.\r\n", ret);
    return ret;
  }

  ck_letter_t letter = {
      .envelope = kings_envelope,
      .page = page,
  };

  ret = ck_process_kings_letter(&letter);
  if (ret != CK_OK) {
    printf("Error: failed to send start command to self: %d.\r\n", ret);
    return ret;
  }

  ret = ck_send_letter(&letter);
  if (ret != CK_OK) {
    printf("Error: failed to send start command: %d\r\n", ret);
    return ret;
  }

  return CK_OK;
}

bool someone_else_is_king(void) {
  return ck_get_comm_mode() != CK_COMM_MODE_SILENT;
}
