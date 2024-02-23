#include "rover-helpers.h"

#include <stdio.h>

#include "rover.h"

// CK
#include "king.h"
#include "mayor.h"
#include "postmaster.h"

#define ASSIGNMENT_COUNT 30

typedef struct {
  uint16_t envelope;
  uint8_t city;
  uint8_t folder;
} rover_assignment_t;

typedef struct {
  rover_assignment_t *assignments;
  uint8_t assignment_count;
} rover_kingdom_t;

static rover_assignment_t assignments[ASSIGNMENT_COUNT];

static rover_kingdom_t kingdom = {
    .assignments = assignments,
    .assignment_count = ASSIGNMENT_COUNT,
};

static ck_envelope_t kings_envelope = {
    .envelope_no = 0,
    .enable = true,
    .is_remote = false,
    .is_compressed = false,
    .has_extended_id = false,
};

static void init_assignments(void);

ck_err_t send_default_letter(void) {
  // Spoof a default letter reception
  ck_err_t err = ck_default_letter_received();
  if (err != CK_OK) {
    printf("Error: failed to receive default letter: %d\r\n", err);
    return err;
  }

  ck_letter_t default_letter = ck_default_letter();
  err = ck_send_letter(&default_letter);
  if (err != CK_OK) {
    printf("Error: failed to send default letter: %d\r\n", err);
    return err;
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
  ck_err_t err = ck_create_kings_page_1(&kp1_args, &page);
  if (err != CK_OK) {
    printf("Error: failed to create king's page: %d.\r\n", err);
    return err;
  }

  ck_letter_t letter = {
      .envelope = kings_envelope,
      .page = page,
  };

  // Send base number to self
  err = ck_process_kings_letter(&letter);
  if (err != CK_OK) {
    printf("Error: couldn't send base number letter to self: %d.\r\n", err);
    return err;
  }

  err = ck_send_letter(&letter);
  if (err != CK_OK) {
    printf("Error: couldn't send base number letter: %d.\r\n", err);
    return err;
  }

  return CK_OK;
}

ck_err_t assign_rover_envelopes(const ck_id_t *own_id) {
  init_assignments();

  for (uint8_t i = 0; i < kingdom.assignment_count; i++) {
    ck_page_t page;
    ck_kp2_args_t kp2_args = {
        .address = kingdom.assignments[i].city,
        .envelope.enable = true,
        .envelope.envelope_no = kingdom.assignments[i].envelope,
        .folder_no = kingdom.assignments[i].folder,
        .envelope_action = CK_ENVELOPE_ASSIGN,
    };
    ck_err_t err = ck_create_kings_page_2(&kp2_args, &page);
    if (err != CK_OK) {
      printf("Error: failed to create king's page: %d.\r\n", err);
      return err;
    }

    ck_letter_t letter = {
        .envelope = kings_envelope,
        .page = page,
    };

    // Letters meant for us need to be processed internally,
    // since we can't receive the messages we send.
    if (kingdom.assignments[i].city == own_id->city_address) {
      err = ck_process_kings_letter(&letter);
      if (err != CK_OK) {
        printf("Error: failed to assign envelope to self: %d.\r\n", err);
        return err;
      }
    }

    err = ck_send_letter(&letter);
    if (err != CK_OK) {
      printf("Error: failed to assign envelope: %d\r\n", err);
      return err;
    }
  }

  return CK_OK;
}

// Only setting so far is the reversal of the servo.
ck_err_t configure_rover_settings(void) {
  ck_envelope_t reverse_envelope = {
      .envelope_no = ROVER_SERVO_REVERSE_ENVELOPE,
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

  ck_err_t err = ck_send_letter(&letter);
  if (err != CK_OK) {
    printf("Error: failed to send servo reverse letter: %d\r\n", err);
    return err;
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
  ck_err_t err = ck_create_kings_page_0(&kp0_args, &page);
  if (err != CK_OK) {
    printf("Error: failed to create king's page: %d.\r\n", err);
    return err;
  }

  ck_letter_t letter = {
      .envelope = kings_envelope,
      .page = page,
  };

  err = ck_process_kings_letter(&letter);
  if (err != CK_OK) {
    printf("Error: failed to send start command to self: %d.\r\n", err);
    return err;
  }

  err = ck_send_letter(&letter);
  if (err != CK_OK) {
    printf("Error: failed to send start command: %d\r\n", err);
    return err;
  }

  return CK_OK;
}

bool someone_else_is_king(void) {
  return ck_get_comm_mode() != CK_COMM_MODE_SILENT;
}

// NOLINTBEGIN(*-magic-numbers)
static void init_assignments(void) {
  int index = 0;

  assignments[index].city = ROVER_SERVO_ID;
  assignments[index].envelope = ROVER_STEERING_ENVELOPE;
  assignments[index].folder = 9;
  index++;

  assignments[index].city = ROVER_MOTOR_ID;
  assignments[index].envelope = ROVER_THROTTLE_ENVELOPE;
  assignments[index].folder = 9;
  index++;

  assignments[index].city = ROVER_SBUS_RECEIVER_ID;
  assignments[index].envelope = ROVER_STEERING_ENVELOPE;
  assignments[index].folder = 2;
  index++;

  assignments[index].city = ROVER_SBUS_RECEIVER_ID;
  assignments[index].envelope = ROVER_THROTTLE_ENVELOPE;
  assignments[index].folder = 3;
  index++;

  assignments[index].city = ROVER_BATTERY_MONITOR_ID;
  assignments[index].envelope = ROVER_BATTERY_CELL_VOLTAGES_ENVELOPE;
  assignments[index].folder = 2;
  index++;

  assignments[index].city = ROVER_BATTERY_MONITOR_ID;
  assignments[index].envelope = ROVER_BATTERY_REGULATED_OUTPUT_ENVELOPE;
  assignments[index].folder = 3;
  index++;

  assignments[index].city = ROVER_BATTERY_MONITOR_ID;
  assignments[index].envelope = ROVER_BATTERY_BATTERY_OUTPUT_ENVELOPE;
  assignments[index].folder = 4;
  index++;

  assignments[index].city = ROVER_SERVO_ID;
  assignments[index].envelope = ROVER_SERVO_VOLTAGE_ENVELOPE;
  assignments[index].folder = 5;
  index++;

  // TODO: Remove this when the power board is standard on the Rover.
  assignments[index].city = ROVER_SERVO_ID;
  assignments[index].envelope = ROVER_BATTERY_VOLTAGE_ENVELOPE;
  assignments[index].folder = 4;
  index++;

  assignments[index].city = ROVER_SERVO_ID;
  assignments[index].envelope = ROVER_SERVO_CURRENT_ENVELOPE;
  assignments[index].folder = 3;
  index++;

  assignments[index].city = ROVER_BATTERY_MONITOR_ID;
  assignments[index].envelope = ROVER_BATTERY_JUMPER_CONFIG_ENVELOPE;
  assignments[index].folder = 5;
  index++;

  assignments[index].city = ROVER_BATTERY_MONITOR_ID;
  assignments[index].envelope = ROVER_BATTERY_REG_OUT_VOLTAGE_ENVELOPE;
  assignments[index].folder = 6;
  index++;

  assignments[index].city = ROVER_BATTERY_MONITOR_ID;
  assignments[index].envelope = ROVER_BATTERY_OUTPUT_ON_OFF_ENVELOPE;
  assignments[index].folder = 7;
  index++;

  assignments[index].city = ROVER_BATTERY_MONITOR_ID;
  assignments[index].envelope = ROVER_BATTERY_REPORT_FREQUENCY_ENVELOPE;
  assignments[index].folder = 8;
  index++;

  assignments[index].city = ROVER_BATTERY_MONITOR_ID;
  assignments[index].envelope = ROVER_BATTERY_LOW_VOLTAGE_CUTOFF_ENVELOPE;
  assignments[index].folder = 9;
  index++;

  assignments[index].city = ROVER_BATTERY_MONITOR_ID;
  assignments[index].envelope =
      ROVER_BATTERY_VBAT_OUT_OVERCURRENT_THRESHOLD_ENVELOPE;
  assignments[index].folder = 10;
  index++;

  assignments[index].city = ROVER_BATTERY_MONITOR_ID;
  assignments[index].envelope =
      ROVER_BATTERY_REG_OUT_OVERCURRENT_THRESHOLD_ENVELOPE;
  assignments[index].folder = 11;
  index++;

  assignments[index].city = ROVER_SERVO_ID;
  assignments[index].envelope = ROVER_SERVO_SET_VOLTAGE_ENVELOPE;
  assignments[index].folder = 7;
  index++;

  assignments[index].city = ROVER_SERVO_ID;
  assignments[index].envelope = ROVER_SERVO_PWM_CONFIG_ENVELOPE;
  assignments[index].folder = 8;
  index++;

  assignments[index].city = ROVER_SERVO_ID;
  assignments[index].envelope = ROVER_SERVO_REPORT_FREQUENCY_ENVELOPE;
  assignments[index].folder = 11;
  index++;

  assignments[index].city = ROVER_MOTOR_ID;
  assignments[index].envelope = ROVER_MOTOR_PWM_CONFIG_ENVELOPE;
  assignments[index].folder = 8;
  index++;

  assignments[index].city = ROVER_SERVO_ID;
  assignments[index].envelope = ROVER_SERVO_REVERSE_ENVELOPE;
  assignments[index].folder = 12;
  index++;

  assignments[index].city = ROVER_MOTOR_ID;
  assignments[index].envelope = ROVER_MOTOR_REVERSE_ENVELOPE;
  assignments[index].folder = 12;
  index++;

  assignments[index].city = ROVER_SERVO_ID;
  assignments[index].envelope = ROVER_SERVO_FAILSAFE_ENVELOPE;
  assignments[index].folder = 13;
  index++;

  assignments[index].city = ROVER_MOTOR_ID;
  assignments[index].envelope = ROVER_MOTOR_FAILSAFE_ENVELOPE;
  assignments[index].folder = 13;
  index++;

  assignments[index].city = ROVER_SBUS_RECEIVER_ID;
  assignments[index].envelope = ROVER_SERVO_SET_SUBTRIM_ENVELOPE;
  assignments[index].folder = 4;
  index++;

  assignments[index].city = ROVER_SBUS_RECEIVER_ID;
  assignments[index].envelope = ROVER_MOTOR_SET_SUBTRIM_ENVELOPE;
  assignments[index].folder = 5;
  index++;

  assignments[index].city = ROVER_SERVO_ID;
  assignments[index].envelope = ROVER_SERVO_SET_SUBTRIM_ENVELOPE;
  assignments[index].folder = 10;
  index++;

  assignments[index].city = ROVER_MOTOR_ID;
  assignments[index].envelope = ROVER_MOTOR_SET_SUBTRIM_ENVELOPE;
  assignments[index].folder = 10;
  index++;

  // Disable for now as it's a work in progress.
  // assignments[index].city = ROVER_SERVO_ID;
  // assignments[index].envelope = ROVER_SERVO_POSITION_ENVELOPE;
  // assignments[index].folder = 2;
  // index++;
}
// NOLINTEND(*-magic-numbers)
