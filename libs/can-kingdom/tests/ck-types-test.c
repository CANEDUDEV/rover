#include "ck-types.h"

#include <string.h>

#include "ck-test.h"
#include "test.h"

void test_check_action_mode(void);
void test_check_comm_mode(void);
void test_check_list_type(void);
void test_check_can_bit_timing(void);
void test_default_letter(void);
void test_check_ck_id(void);

int main(void) {
  test_check_action_mode();
  test_check_comm_mode();
  test_check_list_type();
  test_check_can_bit_timing();
  test_default_letter();
  test_check_ck_id();
}

void test_check_action_mode(void) {
  ASSERT(ck_check_action_mode(CK_ACTION_MODE_KEEP_CURRENT) == CK_OK, "");
  ASSERT(ck_check_action_mode(CK_ACTION_MODE_RUN) == CK_OK, "");
  ASSERT(ck_check_action_mode(CK_ACTION_MODE_FREEZE) == CK_OK, "");
  ASSERT(ck_check_action_mode(CK_ACTION_MODE_RESET) == CK_OK, "");

  const int invalid_mode = 8;
  ASSERT(ck_check_action_mode(invalid_mode) != CK_OK, "");
}

void test_check_comm_mode(void) {
  ASSERT(ck_check_comm_mode(CK_COMM_MODE_KEEP_CURRENT) == CK_OK, "");
  ASSERT(ck_check_comm_mode(CK_COMM_MODE_SILENT) == CK_OK, "");
  ASSERT(ck_check_comm_mode(CK_COMM_MODE_LISTEN_ONLY) == CK_OK, "");
  ASSERT(ck_check_comm_mode(CK_COMM_MODE_COMMUNICATE) == CK_OK, "");

  const int invalid_mode = 8;
  ASSERT(ck_check_comm_mode(invalid_mode) != CK_OK, "");
}

void test_check_list_type(void) {
  ASSERT(ck_check_list_type(CK_LIST_BIT) == CK_OK, "");
  ASSERT(ck_check_list_type(CK_LIST_LINE) == CK_OK, "");
  ASSERT(ck_check_list_type(CK_LIST_PAGE) == CK_OK, "");
  ASSERT(ck_check_list_type(CK_LIST_DOCUMENT) == CK_OK, "");

  const int invalid_type = 8;
  ASSERT(ck_check_list_type(invalid_type) != CK_OK, "");
}

void test_check_can_bit_timing(void) {
  ck_can_bit_timing_t bit_timing = ck_default_bit_timing();
  ASSERT(ck_check_can_bit_timing(&bit_timing) == CK_OK, "");

  ck_can_bit_timing_t illegal_prescaler = bit_timing;
  illegal_prescaler.prescaler = 0;
  ASSERT(ck_check_can_bit_timing(&illegal_prescaler) != CK_OK, "");

  ck_can_bit_timing_t illegal_tq = bit_timing;
  illegal_tq.time_quanta = 0;
  ASSERT(ck_check_can_bit_timing(&illegal_tq) != CK_OK, "");

  ck_can_bit_timing_t illegal_ps2 = bit_timing;
  illegal_ps2.phase_seg2 = 0;
  ASSERT(ck_check_can_bit_timing(&illegal_ps2) != CK_OK, "");

  ck_can_bit_timing_t illegal_sjw = bit_timing;
  illegal_sjw.sjw = 0;
  ASSERT(ck_check_can_bit_timing(&illegal_sjw) != CK_OK, "");
}

void test_default_letter(void) {
  ck_letter_t letter = ck_default_letter();
  ASSERT(letter.envelope.envelope_no == CK_DEFAULT_LETTER_ENVELOPE,
         "incorrect envelope number, expected: %u, got: %u",
         CK_DEFAULT_LETTER_ENVELOPE, letter.envelope.envelope_no);

  const uint8_t expected_data[CK_MAX_LINES_PER_PAGE] = {0xAA, 0xAA, 0xAA, 0xAA,
                                                        0xAA, 0xAA, 0xAA, 0xAA};

  if (memcmp(letter.page.lines, expected_data, sizeof(expected_data)) != 0) {
    printf("default_letter: incorrect page data, expected:");
    for (uint8_t i = 0; i < CK_MAX_LINES_PER_PAGE; i++) {
      printf(" %u,", expected_data[i]);
    }
    printf(" got:");
    for (uint8_t i = 0; i < letter.page.line_count; i++) {
      printf(" %u,", letter.page.lines[i]);
    }
    printf(".");

    ASSERT(false, "");
  }
}

void test_check_ck_id(void) {
  ck_id_t invalid_city_address = {
      .city_address = 0,
  };
  ASSERT(ck_check_ck_id(&invalid_city_address) != CK_OK, "");
  ck_id_t invalid_base_no = {
      .base_no = illegal_can_std_id,
  };
  ASSERT(ck_check_ck_id(&invalid_base_no) != CK_OK, "");
  ck_id_t invalid_base_no_ext = {
      .base_no = illegal_can_ext_id,
      .base_no_has_extended_id = true,
  };
  ASSERT(ck_check_ck_id(&invalid_base_no_ext) != CK_OK, "");
}
