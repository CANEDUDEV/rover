#include "types.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "test.h"

static test_err_t test_check_action_mode(void);
static test_err_t test_check_comm_mode(void);
static test_err_t test_check_list_type(void);
static test_err_t test_check_can_bit_timing(void);
static test_err_t test_default_letter(void);

int main(void) {
  assert(test_check_action_mode() == TEST_PASS);
  assert(test_check_comm_mode() == TEST_PASS);
  assert(test_check_list_type() == TEST_PASS);
  assert(test_check_can_bit_timing() == TEST_PASS);
  assert(test_default_letter() == TEST_PASS);
}

static test_err_t test_check_action_mode(void) {
  ck_action_mode_t mode = CK_ACTION_MODE_KEEP_CURRENT;
  if (ck_check_action_mode(mode) != CK_OK) {
    printf("check_action_mode: valid mode returns error.\n");
    return TEST_FAIL;
  }
  mode = CK_ACTION_MODE_RUN;
  if (ck_check_action_mode(mode) != CK_OK) {
    printf("check_action_mode: valid mode returns error.\n");
    return TEST_FAIL;
  }
  mode = CK_ACTION_MODE_FREEZE;
  if (ck_check_action_mode(mode) != CK_OK) {
    printf("check_action_mode: valid mode returns error.\n");
    return TEST_FAIL;
  }
  mode = CK_ACTION_MODE_RESET;
  if (ck_check_action_mode(mode) != CK_OK) {
    printf("check_action_mode: valid mode returns error.\n");
    return TEST_FAIL;
  }
  const int invalid_mode = 8;
  mode = invalid_mode;
  if (ck_check_action_mode(mode) == CK_OK) {
    printf("check_action_mode: invalid mode returns OK.\n");
    return TEST_FAIL;
  }
  return TEST_PASS;
}

static test_err_t test_check_comm_mode(void) {
  ck_comm_mode_t mode = CK_COMM_MODE_KEEP_CURRENT;
  if (ck_check_comm_mode(mode) != CK_OK) {
    printf("check_comm_mode: valid mode returns error.\n");
    return TEST_FAIL;
  }
  mode = CK_COMM_MODE_SILENT;
  if (ck_check_comm_mode(mode) != CK_OK) {
    printf("check_comm_mode: valid mode returns error.\n");
    return TEST_FAIL;
  }
  mode = CK_COMM_MODE_LISTEN_ONLY;
  if (ck_check_comm_mode(mode) != CK_OK) {
    printf("check_comm_mode: valid mode returns error.\n");
    return TEST_FAIL;
  }
  mode = CK_COMM_MODE_COMMUNICATE;
  if (ck_check_comm_mode(mode) != CK_OK) {
    printf("check_comm_mode: valid mode returns error.\n");
    return TEST_FAIL;
  }
  const int invalid_mode = 8;
  mode = invalid_mode;
  if (ck_check_comm_mode(mode) == CK_OK) {
    printf("check_comm_mode: invalid mode returns OK.\n");
    return TEST_FAIL;
  }
  return TEST_PASS;
}

static test_err_t test_check_list_type(void) {
  ck_list_type_t type = CK_LIST_BIT;
  if (ck_check_list_type(type) != CK_OK) {
    printf("check_list_type: valid type returns error.\n");
    return TEST_FAIL;
  }
  type = CK_LIST_LINE;
  if (ck_check_list_type(type) != CK_OK) {
    printf("check_list_type: valid type returns error.\n");
    return TEST_FAIL;
  }
  type = CK_LIST_PAGE;
  if (ck_check_list_type(type) != CK_OK) {
    printf("check_list_type: valid type returns error.\n");
    return TEST_FAIL;
  }
  type = CK_LIST_DOCUMENT;
  if (ck_check_list_type(type) != CK_OK) {
    printf("check_list_type: valid type returns error.\n");
    return TEST_FAIL;
  }
  const int invalid_type = 8;
  type = invalid_type;
  if (ck_check_list_type(type) == CK_OK) {
    printf("check_list_type: invalid type returns OK.\n");
    return TEST_FAIL;
  }
  return TEST_PASS;
}

static test_err_t test_check_can_bit_timing(void) {
  ck_can_bit_timing_t bit_timing = ck_default_bit_timing();
  if (ck_check_can_bit_timing(&bit_timing) != CK_OK) {
    printf("check_can_bit_timing: valid bit timing returns error.\n");
    return TEST_FAIL;
  }

  // Illegal parameters test
  ck_can_bit_timing_t illegal_prescaler = bit_timing;
  ck_can_bit_timing_t illegal_tq = bit_timing;
  ck_can_bit_timing_t illegal_ps2 = bit_timing;
  ck_can_bit_timing_t illegal_sjw = bit_timing;

  illegal_prescaler.prescaler = 0;
  illegal_tq.time_quanta = 0;
  illegal_ps2.phase_seg2 = 0;
  illegal_sjw.sjw = 0;

  if (ck_check_can_bit_timing(&illegal_prescaler) == CK_OK) {
    printf("check_can_bit_timing: illegal prescaler returned OK.\n");
    return TEST_FAIL;
  }
  if (ck_check_can_bit_timing(&illegal_tq) == CK_OK) {
    printf("check_can_bit_timing: illegal time quanta returned OK.\n");
    return TEST_FAIL;
  }
  if (ck_check_can_bit_timing(&illegal_ps2) == CK_OK) {
    printf("check_can_bit_timing: illegal phase seg 2 returned OK.\n");
    return TEST_FAIL;
  }
  if (ck_check_can_bit_timing(&illegal_sjw) == CK_OK) {
    printf("check_can_bit_timing: illegal sjw returned OK.\n");
    return TEST_FAIL;
  }

  return TEST_PASS;
}

static test_err_t test_default_letter(void) {
  ck_letter_t letter = ck_default_letter();
  if (letter.envelope.envelope_no != CK_DEFAULT_LETTER_ENVELOPE) {
    printf("default_letter: incorrect envelope number, expected: %u, got: %u\n",
           CK_DEFAULT_LETTER_ENVELOPE, letter.envelope.envelope_no);
    return TEST_FAIL;
  }
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
    printf(".\n");
    return TEST_FAIL;
  }
  return TEST_PASS;
}
