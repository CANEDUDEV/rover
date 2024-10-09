#include "mayor.h"
#include "rover-defs.h"
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

void setup_test(void);
void test_send_default_letter(void);
void test_set_rover_base_number(void);
void test_assign_rover_envelopes(void);
void test_start_communication(void);

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
  setup_test();
  test_send_default_letter();
  test_set_rover_base_number();
  test_assign_rover_envelopes();
  test_start_communication();
}

void setup_test(void) {
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

  if (ck_mayor_init(&mayor) != CK_OK) {
    exit(TEST_SETUP_FAIL);
  }
}

void test_send_default_letter(void) {
  ASSERT(ck_get_comm_mode() == CK_COMM_MODE_SILENT,
         "comm mode must be SILENT before default letter is sent.");

  ASSERT(send_default_letter() == CK_OK, "");

  ASSERT(ck_get_comm_mode() == CK_COMM_MODE_LISTEN_ONLY,
         "comm mode must be LISTEN_ONLY after default letter is sent.");
}

void test_set_rover_base_number(void) {
  ASSERT(set_rover_base_number() == CK_OK, "");

  uint32_t base_no = ck_get_base_number();
  ASSERT(base_no == ROVER_BASE_NUMBER, "");
}

void test_assign_rover_envelopes(void) {
  uint8_t envelope_counts_before[CK_DATA_FOLDER_COUNT];
  for (int i = 0; i < CK_DATA_FOLDER_COUNT; i++) {
    envelope_counts_before[i] = ck_data.folders[i].envelope_count;
  }

  ASSERT(assign_rover_envelopes(&ck_id) == CK_OK, "");

  for (int i = 0; i < CK_DATA_FOLDER_COUNT; i++) {
    uint8_t envelope_count_after = ck_data.folders[i].envelope_count;
    ASSERT(envelope_counts_before[i] == envelope_count_after,
           "folder %d was modified. Envelope count before: %u, "
           "envelope count after:  %u",
           i, envelope_counts_before[i], envelope_count_after);
  }
}

void test_start_communication(void) {
  ASSERT(ck_get_comm_mode() == CK_COMM_MODE_LISTEN_ONLY,
         "comm mode must be LISTEN_ONLY before communication starts.");

  ASSERT(start_communication() == CK_OK, "");

  ASSERT(ck_get_comm_mode() == CK_COMM_MODE_COMMUNICATE,
         "comm mode must be COMMUNICATE after communication starts.");
}
