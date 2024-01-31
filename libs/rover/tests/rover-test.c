#include "rover.h"

#include <assert.h>
#include <stdio.h>

#include "mayor.h"
#include "rover-helpers.h"
#include "test.h"

#define CK_DATA_FOLDER_COUNT 2
#define CK_DATA_LIST_COUNT 2

typedef struct {
  ck_folder_t folders[CK_DATA_FOLDER_COUNT];
  ck_list_t lists[CK_DATA_LIST_COUNT];
} ck_data_t;

static ck_data_t ck_data;
static const ck_id_t ck_id = {.city_address = 99};

test_err_t setup_test(void);
test_err_t test_send_default_letter(void);
test_err_t test_set_rover_base_number(void);
test_err_t test_assign_rover_envelopes(void);
test_err_t test_start_communication(void);

ck_err_t mock_set_action_mode(ck_action_mode_t mode) {
  (void)mode;
  return CK_OK;
}

ck_err_t mock_set_city_mode(ck_city_mode_t mode) {
  (void)mode;
  return CK_OK;
}

void mock_start_200ms_timer(void) {
}

int main(void) {
  assert(setup_test() == TEST_PASS);
  assert(test_send_default_letter() == TEST_PASS);
  assert(test_set_rover_base_number() == TEST_PASS);
  assert(test_assign_rover_envelopes() == TEST_PASS);
  assert(test_start_communication() == TEST_PASS);
}

test_err_t setup_test(void) {
  ck_data.lists[0].type = CK_LIST_DOCUMENT;
  ck_data.lists[0].direction = CK_DIRECTION_TRANSMIT;
  ck_data.lists[0].list_no = 0;

  ck_data.lists[1].type = CK_LIST_DOCUMENT;
  ck_data.lists[1].direction = CK_DIRECTION_RECEIVE;
  ck_data.lists[1].list_no = 0;

  ck_mayor_t mayor = {
      .ean_no = ck_id.city_address,
      .serial_no = ck_id.city_address,
      .ck_id = ck_id,
      .set_action_mode = mock_set_action_mode,
      .set_city_mode = mock_set_city_mode,
      .start_200ms_timer = mock_start_200ms_timer,
      .folder_count = CK_DATA_FOLDER_COUNT,
      .folders = ck_data.folders,
      .list_count = CK_DATA_LIST_COUNT,
      .lists = ck_data.lists,
  };

  ck_err_t err = ck_mayor_init(&mayor);
  if (err != CK_OK) {
    fprintf(stderr, "Error: failed to init mayor: %d\n", err);
    return TEST_SETUP_FAIL;
  }

  return TEST_PASS;
}

test_err_t test_send_default_letter(void) {
  if (ck_get_comm_mode() != CK_COMM_MODE_SILENT) {
    fprintf(stderr,
            "Error: comm mode must be SILENT before default letter is sent.\n");
    return TEST_FAIL;
  }

  ck_err_t err = send_default_letter();
  if (err != CK_OK) {
    fprintf(stderr, "Error: failed to send default letter: %d.\n", err);
    return TEST_FAIL;
  }

  if (ck_get_comm_mode() != CK_COMM_MODE_LISTEN_ONLY) {
    fprintf(
        stderr,
        "Error: comm mode must be LISTEN_ONLY after default letter is sent.\n");
    return TEST_FAIL;
  }

  return TEST_PASS;
}

test_err_t test_set_rover_base_number(void) {
  ck_err_t err = set_rover_base_number();
  if (err != CK_OK) {
    fprintf(stderr, "Error: failed to set base number: %d\n", err);
    return TEST_FAIL;
  }

  uint32_t base_no = ck_get_base_number();
  if (base_no != ROVER_BASE_NUMBER) {
    fprintf(stderr, "Error: wrong base number, expected: %u, actual: %u\n",
            ROVER_BASE_NUMBER, base_no);
    return TEST_FAIL;
  }

  return TEST_PASS;
}

test_err_t test_assign_rover_envelopes(void) {
  uint8_t envelope_counts_before[CK_DATA_FOLDER_COUNT];
  for (int i = 0; i < CK_DATA_FOLDER_COUNT; i++) {
    envelope_counts_before[i] = ck_data.folders[i].envelope_count;
  }

  ck_err_t err = assign_rover_envelopes(&ck_id);
  if (err != CK_OK) {
    fprintf(stderr, "Error: failed to assign envelopes: %d \n", err);
    return TEST_FAIL;
  }

  for (int i = 0; i < CK_DATA_FOLDER_COUNT; i++) {
    uint8_t envelope_count_after = ck_data.folders[i].envelope_count;
    if (envelope_counts_before[i] != envelope_count_after) {
      fprintf(stderr,
              "Error: folder no %d was modified. Envelope count before: %u, "
              "envelope count after:  %u\n",
              i, envelope_counts_before[i], envelope_count_after);
      return TEST_FAIL;
    }
  }

  return TEST_PASS;
}

test_err_t test_start_communication(void) {
  if (ck_get_comm_mode() != CK_COMM_MODE_LISTEN_ONLY) {
    fprintf(
        stderr,
        "Error: comm mode must be LISTEN_ONLY before communication starts.\n");
    return TEST_FAIL;
  }

  ck_err_t err = start_communication();
  if (err != CK_OK) {
    fprintf(stderr, "Error: failed to start communication: %d\n", err);
    return TEST_FAIL;
  }

  if (ck_get_comm_mode() != CK_COMM_MODE_COMMUNICATE) {
    fprintf(
        stderr,
        "Error: comm mode must be COMMUNICATE after communication starts.\n");
    return TEST_FAIL;
  }

  return TEST_PASS;
}
