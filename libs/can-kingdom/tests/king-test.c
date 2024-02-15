#include "king.h"

#include <string.h>

#include "ck-test.h"
#include "test.h"

void test_kp0(void);
void test_kp1(void);
void test_kp2(void);
void test_kp8(void);
void test_kp16(void);
void test_kp17(void);
void verify_kp_header(int page_no, ck_page_t *page);

int main(void) {
  test_kp0();
  test_kp1();
  test_kp2();
  test_kp8();
  test_kp16();
  test_kp17();
}

// NOLINTBEGIN(*-magic-numbers)
void test_kp0(void) {
  ck_kp0_args_t args = {
      .address = test_city_address,
      .action_mode = CK_ACTION_MODE_RUN,
      .comm_mode = CK_COMM_MODE_COMMUNICATE,
      .comm_flags = CK_COMM_SKIP_LISTEN | CK_COMM_SKIP_WAIT | CK_COMM_RESET,
      .city_mode = CK_CITY_MODE_KEEP_CURRENT,
  };

  ck_page_t page;

  ASSERT(ck_create_kings_page_0(&args, &page) == CK_OK, "");

  verify_kp_header(0, &page);

  ASSERT(page.lines[2] == args.action_mode,
         "wrong action mode, expected: %x, got: %x.", args.action_mode,
         page.lines[2]);

  ck_comm_mode_t comm_mode = page.lines[3] & 0x3;

  ASSERT(comm_mode == args.comm_mode, "wrong comm mode, expected: %x, got: %x.",
         args.comm_mode, comm_mode);

  ck_comm_flags_t comm_flags = page.lines[3] & ~0x3;

  ASSERT(comm_flags == args.comm_flags,
         "wrong flags set, expected: %x, got: %x.", args.comm_flags,
         comm_flags);

  ASSERT(page.lines[4] == args.city_mode,
         "wrong city mode, expected: %x, got: %x.", args.city_mode,
         page.lines[4]);
}

void test_kp1(void) {
  ck_kp1_args_t args = {
      .address = test_city_address,
      .mayor_response_no = CK_NO_RESPONSE_REQUESTED,
      .base_no = test_base_no,
      .has_extended_id = true,
  };
  ck_page_t page;

  ASSERT(ck_create_kings_page_1(&args, &page) == CK_OK, "");

  verify_kp_header(1, &page);

  ASSERT(page.lines[2] == CK_NO_RESPONSE_REQUESTED,
         "wrong mayor's response page, expected: %u, got: %u.",
         CK_NO_RESPONSE_REQUESTED, page.lines[2]);

  uint32_t got_base_no = 0;
  memcpy(&got_base_no, &page.lines[4], sizeof(got_base_no));
  got_base_no &= CK_CAN_ID_MASK;

  ASSERT(got_base_no == args.base_no,
         "wrong base number, expected: %u, got: %u.", args.base_no,
         got_base_no);

  bool got_has_extended_id = (page.lines[7] >> 7) & 0x01;

  ASSERT(got_has_extended_id == args.has_extended_id,
         "wrong extended ID flag, expected: %u, got: %u.", args.has_extended_id,
         got_has_extended_id);

  // Test illegal base numbers.
  args.base_no = illegal_can_std_id;
  args.has_extended_id = false;
  memset(&page, 0, sizeof(page));

  ASSERT(ck_create_kings_page_1(&args, &page) == CK_ERR_INVALID_CAN_ID,
         "setting illegal standard base number should fail.");

  args.base_no = illegal_can_ext_id;
  args.has_extended_id = true;
  memset(&page, 0, sizeof(page));

  ASSERT(ck_create_kings_page_1(&args, &page) == CK_ERR_INVALID_CAN_ID,
         "setting illegal extended base number should fail.");
}

void test_kp2(void) {
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
  ASSERT(ck_create_kings_page_2(&args, &page) == CK_OK, "");

  verify_kp_header(2, &page);

  uint32_t got_envelope_no = 0;
  memcpy(&got_envelope_no, &page.lines[2], sizeof(got_envelope_no));
  got_envelope_no &= CK_CAN_ID_MASK;

  ASSERT(got_envelope_no == args.envelope.envelope_no,
         "wrong envelope number, expected: %u, got: %u.",
         args.envelope.envelope_no, got_envelope_no);

  bool got_has_extended_id = (page.lines[5] >> 7) & 0x01;

  ASSERT(got_has_extended_id == args.envelope.has_extended_id,
         "wrong extended ID flag, expected: %u, got: %u.",
         args.envelope.has_extended_id, got_has_extended_id);

  ASSERT(page.lines[6] == args.folder_no,
         "wrong folder number, expected: %u, got: %u.", args.folder_no,
         page.lines[6]);

  bool got_enable_envelope = page.lines[7] & 0x01;

  ASSERT(got_enable_envelope == args.envelope.enable,
         "wrong enable envelope flag, expected: %u, got: %u.",
         args.envelope.enable, got_enable_envelope);

  ck_envelope_action_t got_envelope_action =
      (page.lines[7] >> 1) & 0x03;  // Two bits

  ASSERT(got_envelope_action == args.envelope_action,
         "wrong envelope action, expected: %u, got: %u.", args.envelope_action,
         got_envelope_action);

  // Test illegal envelope numbers
  args.envelope.envelope_no = illegal_can_std_id;
  args.envelope.has_extended_id = false;
  memset(&page, 0, sizeof(page));

  ASSERT(ck_create_kings_page_2(&args, &page) == CK_ERR_INVALID_CAN_ID,
         "setting illegal standard envelope number should fail.");

  args.envelope.envelope_no = illegal_can_ext_id;
  args.envelope.has_extended_id = true;
  memset(&page, 0, sizeof(page));

  ASSERT(ck_create_kings_page_2(&args, &page) == CK_ERR_INVALID_CAN_ID,
         "setting illegal extended envelope number should fail.");

  // Test illegal parameter combination (enable = true && expel)
  args.envelope.envelope_no = 456;
  args.envelope.enable = true;
  args.envelope_action = CK_ENVELOPE_EXPEL;
  memset(&page, 0, sizeof(page));

  ASSERT(ck_create_kings_page_2(&args, &page) == CK_ERR_INCOMPATIBLE_PARAMS,
         "setting illegal parameter combination should fail.");
}

void test_kp8(void) {
  ck_page_t page;
  ck_can_bit_timing_t default_bit_timing = ck_default_bit_timing();

  ASSERT(ck_create_kings_page_8(test_city_address, &default_bit_timing,
                                &page) == CK_OK,
         "failed to set default bit timing.");
}

void test_kp16(void) {
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

  ASSERT(ck_create_kings_page_16(&args, &page) == CK_OK, "");

  verify_kp_header(16, &page);

  uint8_t got_dlc = page.lines[3] & 0x0F;

  ASSERT(got_dlc == args.dlc, "wrong DLC, expected: %u, got: %u.", args.dlc,
         got_dlc);

  bool got_has_rtr = (page.lines[3] >> 6) & 0x01;

  ASSERT(got_has_rtr == args.has_rtr, "wrong has_rtr, expected: %u, got: %u.",
         args.has_rtr, got_has_rtr);

  ck_direction_t got_direction = page.lines[4] & 0x01;

  ASSERT(got_direction == args.direction,
         "wrong direction, expected: %u, got: %u.", args.direction,
         got_direction);

  ck_document_action_t got_document_action =
      (page.lines[4] >> 4) & 0x03;  // Two bits

  ASSERT(got_document_action == args.document_action,
         "wrong document action, expected: %u, got: %u.", args.document_action,
         got_document_action);

  bool got_enable_folder = (page.lines[4] >> 6) & 0x01;

  ASSERT(got_enable_folder == args.enable_folder,
         "KP16: wrong enable folder flag, expected: %u, got: %u.",
         args.enable_folder, got_enable_folder);

  ASSERT(page.lines[5] == args.list_no,
         "wrong list number, expected: %u, got: %u.", args.list_no,
         page.lines[5]);

  ASSERT(page.lines[6] == args.document_no,
         "wrong document number, expected: %u, got: %u.", args.document_no,
         page.lines[6]);

  // Test illegal folder number
  args.folder_no = 1;
  memset(&page, 0, sizeof(page));

  ASSERT(ck_create_kings_page_16(&args, &page) == CK_ERR_INVALID_FOLDER_NUMBER,
         "setting illegal folder number should fail.");

  // Test illegal DLC
  args.folder_no = 78;
  args.dlc = CK_CAN_MAX_DLC + 1;
  memset(&page, 0, sizeof(page));

  ASSERT(ck_create_kings_page_16(&args, &page) == CK_ERR_INVALID_CAN_DLC,
         "setting illegal DLC should fail.");
}

void test_kp17(void) {
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

  ASSERT(ck_create_kings_page_17(&args, &page) == CK_OK, "");

  verify_kp_header(17, &page);

  ck_direction_t got_direction = page.lines[2] & 0x01;

  ASSERT(got_direction == args.direction,
         "wrong direction, expected: %u, got: %u.", args.direction,
         got_direction);

  ck_list_type_t got_list_type = (page.lines[2] >> 3) & 0x03;  // Two bits

  ASSERT(got_list_type == args.list_type,
         "wrong list type, expected: %x, got: %x.", args.list_type,
         got_list_type);

  ASSERT(page.lines[3] == args.source_list_no,
         "wrong source list number, expected: %u, got: %u.",
         args.source_list_no, page.lines[3]);

  ASSERT(page.lines[4] == args.source_record_no,
         "wrong source record number, expected: %u, got: %u.",
         args.source_record_no, page.lines[4]);

  ASSERT(page.lines[5] == args.target_list_no,
         "wrong target list number, expected: %u, got: %u.",
         args.target_list_no, page.lines[5]);

  ASSERT(page.lines[6] == args.target_record_no,
         "wrong target record number, expected: %u, got: %u.",
         args.target_record_no, page.lines[6]);

  ASSERT(page.lines[7] == args.position,
         "wrong position, expected: %u, got: %u.", args.position,
         page.lines[7]);
}

// These verifications are the same for all king's pages.
void verify_kp_header(int page_no, ck_page_t *page) {
  ASSERT(page->line_count == CK_MAX_LINES_PER_PAGE,
         "KP%d: wrong line count, expected: %u, got: %u.", page_no,
         CK_MAX_LINES_PER_PAGE, page->line_count);

  ASSERT(page->lines[0] == test_city_address,
         "KP%d: wrong address, expected: %u, got: %u.", page_no,
         test_city_address, page->lines[0]);

  ASSERT(page->lines[1] == page_no,
         "KP%d: wrong page number, expected: %d, got: %u.", page_no, page_no,
         page->lines[1]);
}
// NOLINTEND(*-magic-numbers)
