#include "king.h"

#include <stdio.h>
#include <string.h>

#include "test.h"

static test_err_t verify_kp_header(int page_no, ck_page_t *page);

static test_err_t test_kp0(void);
static test_err_t test_kp1(void);
static test_err_t test_kp2(void);
static test_err_t test_kp16(void);
static test_err_t test_kp17(void);

int main(void) {
  if (test_kp0() != TEST_PASS) {
    return TEST_FAIL;
  }
  if (test_kp1() != TEST_PASS) {
    return TEST_FAIL;
  }
  if (test_kp2() != TEST_PASS) {
    return TEST_FAIL;
  }
  if (test_kp16() != TEST_PASS) {
    return TEST_FAIL;
  }
  if (test_kp17() != TEST_PASS) {
    return TEST_FAIL;
  }
  return TEST_PASS;
}

// NOLINTBEGIN(*-magic-numbers)
static test_err_t test_kp0(void) {
  ck_kp0_args_t args = {
      .address = test_city_address,
      .action_mode = CK_ACTION_MODE_RUN,
      .comm_mode = CK_COMM_MODE_COMMUNICATE,
      .city_mode = CK_CITY_MODE_KEEP_CURRENT,
  };

  ck_page_t page;

  if (ck_create_kings_page_0(&args, &page) != CK_OK) {
    printf("KP0: failed to create king's page.\n");
    return TEST_FAIL;
  }
  if (verify_kp_header(0, &page) != TEST_PASS) {
    return TEST_FAIL;
  }

  if (page.lines[2] != args.action_mode) {
    printf("KP0: wrong action mode, expected: %x, got: %x.\n", args.action_mode,
           page.lines[2]);
    return TEST_FAIL;
  }
  if (page.lines[3] != args.comm_mode) {
    printf("KP0: wrong comm mode, expected: %x, got: %x.\n", args.comm_mode,
           page.lines[3]);
    return TEST_FAIL;
  }
  if (page.lines[4] != args.city_mode) {
    printf("KP0: wrong city mode, expected: %x, got: %x.\n", args.city_mode,
           page.lines[4]);
    return TEST_FAIL;
  }

  return TEST_PASS;
}

static test_err_t test_kp1(void) {
  ck_kp1_args_t args = {
      .address = test_city_address,
      .mayor_response_no = CK_NO_RESPONSE_REQUESTED,
      .base_no = test_base_no,
      .has_extended_id = true,
  };
  ck_page_t page;
  if (ck_create_kings_page_1(&args, &page) != CK_OK) {
    printf("KP1: failed to create king's page.\n");
    return TEST_FAIL;
  }
  if (verify_kp_header(1, &page) != TEST_PASS) {
    return TEST_FAIL;
  }
  if (page.lines[2] != CK_NO_RESPONSE_REQUESTED) {
    printf("KP1: wrong mayor's response page, expected: %u, got: %u.\n",
           CK_NO_RESPONSE_REQUESTED, page.lines[2]);
    return TEST_FAIL;
  }

  uint32_t got_base_no = 0;
  memcpy(&got_base_no, &page.lines[4], sizeof(got_base_no));
  got_base_no &= CK_CAN_ID_MASK;
  if (got_base_no != args.base_no) {
    printf("KP1: wrong base number, expected: %u, got: %u.\n", args.base_no,
           got_base_no);
    return TEST_FAIL;
  }

  bool got_has_extended_id = (page.lines[7] >> 7) & 0x01;
  if (got_has_extended_id != args.has_extended_id) {
    printf("KP1: wrong extended ID flag, expected: %u, got: %u.\n",
           args.has_extended_id, got_has_extended_id);
    return TEST_FAIL;
  }

  // Test illegal base numbers.
  args.base_no = illegal_can_std_id;
  args.has_extended_id = false;
  memset(&page, 0, sizeof(page));
  if (ck_create_kings_page_1(&args, &page) != CK_ERR_INVALID_CAN_ID) {
    printf("KP1: setting illegal standard base number succeeded.\n");
    return TEST_FAIL;
  }

  args.base_no = illegal_can_ext_id;
  args.has_extended_id = true;
  memset(&page, 0, sizeof(page));
  if (ck_create_kings_page_1(&args, &page) != CK_ERR_INVALID_CAN_ID) {
    printf("KP1: setting illegal extended base number succeeded.\n");
    return TEST_FAIL;
  }

  return TEST_PASS;
}

static test_err_t test_kp2(void) {
  ck_kp2_args_t args = {
      .address = test_city_address,
      .envelope =
          {
              .envelope_no = 456,
              .has_extended_id = true,
              .enable = true,
          },
      .folder_no = 78,
      .envelope_action = CK_ENVELOPE_ASSIGN,
  };

  ck_page_t page;
  if (ck_create_kings_page_2(&args, &page) != CK_OK) {
    printf("KP2: failed to create king's page.\n");
    return TEST_FAIL;
  }
  if (verify_kp_header(2, &page) != TEST_PASS) {
    return TEST_FAIL;
  }

  uint32_t got_envelope_no = 0;
  memcpy(&got_envelope_no, &page.lines[2], sizeof(got_envelope_no));
  got_envelope_no &= CK_CAN_ID_MASK;
  if (got_envelope_no != args.envelope.envelope_no) {
    printf("KP2: wrong envelope number, expected: %u, got: %u.\n",
           args.envelope.envelope_no, got_envelope_no);
    return TEST_FAIL;
  }

  bool got_has_extended_id = (page.lines[5] >> 7) & 0x01;
  if (got_has_extended_id != args.envelope.has_extended_id) {
    printf("KP2: wrong extended ID flag, expected: %u, got: %u.\n",
           args.envelope.has_extended_id, got_has_extended_id);
    return TEST_FAIL;
  }

  if (page.lines[6] != args.folder_no) {
    printf("KP2: wrong folder number, expected: %u, got: %u.\n", args.folder_no,
           page.lines[6]);
    return TEST_FAIL;
  }

  bool got_enable_envelope = page.lines[7] & 0x01;
  if (got_enable_envelope != args.envelope.enable) {
    printf("KP2: wrong enable envelope flag, expected: %u, got: %u.\n",
           args.envelope.enable, got_enable_envelope);
    return TEST_FAIL;
  }

  ck_envelope_action_t got_envelope_action =
      (page.lines[7] >> 1) & 0x03;  // Two bits
  if (got_envelope_action != args.envelope_action) {
    printf("KP2: wrong envelope action, expected: %u, got: %u.\n",
           args.envelope_action, got_envelope_action);
    return TEST_FAIL;
  }

  // Test illegal envelope numbers
  args.envelope.envelope_no = illegal_can_std_id;
  args.envelope.has_extended_id = false;
  memset(&page, 0, sizeof(page));
  if (ck_create_kings_page_2(&args, &page) != CK_ERR_INVALID_CAN_ID) {
    printf("KP2: setting illegal standard envelope number succeeded.\n");
    return TEST_FAIL;
  }

  args.envelope.envelope_no = illegal_can_ext_id;
  args.envelope.has_extended_id = true;
  memset(&page, 0, sizeof(page));
  if (ck_create_kings_page_2(&args, &page) != CK_ERR_INVALID_CAN_ID) {
    printf("KP2: setting illegal extended envelope number succeeded.\n");
    return TEST_FAIL;
  }

  // Test illegal parameter combination (enable = true && expel)
  args.envelope.envelope_no = 456;
  args.envelope.enable = true;
  args.envelope_action = CK_ENVELOPE_EXPEL;
  memset(&page, 0, sizeof(page));
  if (ck_create_kings_page_2(&args, &page) != CK_ERR_INCOMPATIBLE_PARAMS) {
    printf("KP2: setting illegal parameter combination succeeded.\n");
    return TEST_FAIL;
  }

  return TEST_PASS;
}

static test_err_t test_kp16(void) {
  ck_kp16_args_t args = {
      .address = test_city_address,
      .folder_no = 78,
      .has_rtr = true,
      .dlc = CK_CAN_MAX_DLC,
      .direction = CK_DIRECTION_TRANSMIT,
      .document_action = CK_DOCUMENT_INSERT,
      .enable_folder = true,
      .list_no = 5,
      .document_no = 8,
  };

  ck_page_t page;
  if (ck_create_kings_page_16(&args, &page) != CK_OK) {
    printf("KP16: failed to create king's page.\n");
    return TEST_FAIL;
  }
  if (verify_kp_header(16, &page) != TEST_PASS) {
    return TEST_FAIL;
  }

  uint8_t got_dlc = page.lines[3] & 0x0F;
  if (got_dlc != args.dlc) {
    printf("KP16: wrong DLC, expected: %u, got: %u.\n", args.dlc, got_dlc);
    return TEST_FAIL;
  }

  bool got_has_rtr = (page.lines[3] >> 6) & 0x01;
  if (got_has_rtr != args.has_rtr) {
    printf("KP16: wrong has_rtr, expected: %u, got: %u.\n", args.has_rtr,
           got_has_rtr);
    return TEST_FAIL;
  }

  ck_direction_t got_direction = page.lines[4] & 0x01;
  if (got_direction != args.direction) {
    printf("KP16: wrong direction, expected: %u, got: %u.\n", args.direction,
           got_direction);
    return TEST_FAIL;
  }

  ck_document_action_t got_document_action =
      (page.lines[4] >> 4) & 0x03;  // Two bits
  if (got_document_action != args.document_action) {
    printf("KP16: wrong document action, expected: %u, got: %u.\n",
           args.document_action, got_document_action);
    printf("0x%x", page.lines[4]);
    return TEST_FAIL;
  }

  bool got_enable_folder = (page.lines[4] >> 6) & 0x01;
  if (got_enable_folder != args.enable_folder) {
    printf("KP16: wrong enable folder flag, expected: %u, got: %u.\n",
           args.enable_folder, got_enable_folder);
    return TEST_FAIL;
  }

  if (page.lines[5] != args.list_no) {
    printf("KP16: wrong list number, expected: %u, got: %u.\n", args.list_no,
           page.lines[5]);
    return TEST_FAIL;
  }

  if (page.lines[6] != args.document_no) {
    printf("KP16: wrong document number, expected: %u, got: %u.\n",
           args.document_no, page.lines[6]);
    return TEST_FAIL;
  }

  // Test illegal folder number
  args.folder_no = 1;
  memset(&page, 0, sizeof(page));
  if (ck_create_kings_page_16(&args, &page) != CK_ERR_INVALID_FOLDER_NUMBER) {
    printf("KP16: setting illegal folder number succeeded.\n");
    return TEST_FAIL;
  }

  // Test illegal DLC
  args.folder_no = 78;
  args.dlc = CK_CAN_MAX_DLC + 1;
  memset(&page, 0, sizeof(page));
  if (ck_create_kings_page_16(&args, &page) != CK_ERR_INVALID_CAN_DLC) {
    printf("KP16: setting illegal DLC succeeded.\n");
    return TEST_FAIL;
  }

  return TEST_PASS;
}

static test_err_t test_kp17(void) {
  ck_kp17_args_t args = {
      .address = test_city_address,
      .direction = CK_DIRECTION_TRANSMIT,
      .list_type = CK_LIST_PAGE,
      .source_list_no = 1,
      .source_record_no = 2,
      .target_list_no = 3,
      .target_record_no = 4,
      .position = 1,
  };

  ck_page_t page;
  if (ck_create_kings_page_17(&args, &page) != CK_OK) {
    printf("KP17: failed to create king's page.\n");
    return TEST_FAIL;
  }
  if (verify_kp_header(17, &page) != TEST_PASS) {
    return TEST_FAIL;
  }

  ck_direction_t got_direction = page.lines[2] & 0x01;
  if (got_direction != args.direction) {
    printf("KP17: wrong direction, expected: %u, got: %u.\n", args.direction,
           got_direction);
    return TEST_FAIL;
  }

  ck_list_type_t got_list_type = (page.lines[2] >> 3) & 0x03;  // Two bits
  if (got_list_type != args.list_type) {
    printf("KP17: wrong list type, expected: %x, got: %x.\n", args.list_type,
           got_list_type);
    return TEST_FAIL;
  }

  if (page.lines[3] != args.source_list_no) {
    printf("KP17: wrong source list number, expected: %u, got: %u.\n",
           args.source_list_no, page.lines[3]);
    return TEST_FAIL;
  }

  if (page.lines[4] != args.source_record_no) {
    printf("KP17: wrong source record number, expected: %u, got: %u.\n",
           args.source_record_no, page.lines[4]);
    return TEST_FAIL;
  }

  if (page.lines[5] != args.target_list_no) {
    printf("KP17: wrong target list number, expected: %u, got: %u.\n",
           args.target_list_no, page.lines[5]);
    return TEST_FAIL;
  }

  if (page.lines[6] != args.target_record_no) {
    printf("KP17: wrong target record number, expected: %u, got: %u.\n",
           args.target_record_no, page.lines[6]);
    return TEST_FAIL;
  }

  if (page.lines[7] != args.position) {
    printf("KP17: wrong position, expected: %u, got: %u.\n", args.position,
           page.lines[7]);
    return TEST_FAIL;
  }

  return TEST_PASS;
}

// These verifications are the same for all king's pages.
static test_err_t verify_kp_header(int page_no, ck_page_t *page) {
  if (page->line_count != CK_MAX_LINES_PER_PAGE) {
    printf("KP%d: wrong line count, expected: %u, got: %u.\n", page_no,
           CK_MAX_LINES_PER_PAGE, page->line_count);
    return TEST_FAIL;
  }
  if (page->lines[0] != test_city_address) {
    printf("KP%d: wrong address, expected: %u, got: %u.\n", page_no,
           test_city_address, page->lines[0]);
    return TEST_FAIL;
  }
  if (page->lines[1] != page_no) {
    printf("KP%d: wrong page number, expected: %d, got: %u.\n", page_no,
           page_no, page->lines[1]);
    return TEST_FAIL;
  }
  return TEST_PASS;
}

// NOLINTEND(*-magic-numbers)
