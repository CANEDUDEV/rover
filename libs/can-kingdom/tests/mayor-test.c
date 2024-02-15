#include "mayor.h"

#include <string.h>

#include "king.h"
#include "postmaster.h"

// Testing utils
#include "ck-test.h"
#include "test.h"

#define RX_BIT_COUNT 1
#define RX_LINE_COUNT 2
#define RX_PAGE_COUNT 2
#define TX_PAGE_COUNT 3  // Mayor's pages + test page
#define RX_DOCUMENT_COUNT 2
#define TX_DOCUMENT_COUNT 2  // Mayor's document + test doc
#define FOLDER_COUNT 4

#define RX_BIT_ARRAY_LENGTH (1 + RX_BIT_COUNT / 8)

#define BIT_LIST_COUNT 1
#define LINE_LIST_COUNT 1
#define PAGE_LIST_COUNT 1
#define DOC_LIST_COUNT 2
#define LIST_COUNT \
  (BIT_LIST_COUNT + LINE_LIST_COUNT + PAGE_LIST_COUNT + DOC_LIST_COUNT)

struct city_data {
  uint8_t rx_bits[RX_BIT_ARRAY_LENGTH];
  uint8_t rx_lines[RX_LINE_COUNT];
  ck_page_t rx_pages[RX_PAGE_COUNT];
  ck_page_t tx_pages[TX_PAGE_COUNT - 2];  // Don't store mayor's pages
  ck_document_t rx_docs[RX_DOCUMENT_COUNT];
  ck_document_t tx_docs[TX_DOCUMENT_COUNT];
  ck_list_t lists[LIST_COUNT];
  ck_folder_t folders[FOLDER_COUNT];

  // Convenience pointers
  ck_list_t *rx_bit_list;
  ck_list_t *rx_line_list;
  ck_list_t *rx_page_list;
  ck_list_t *rx_doc_list;
  ck_list_t *tx_doc_list;
};

// This mayor's predefined data
static struct city_data data;

void test_process_before_init(void);
void test_mayor_init(void);
void test_add_mayors_page(void);
void test_send_document(void);
void test_send_page(void);
void test_send_mayors_page(void);
void test_is_kings_envelope(void);
void test_get_envelopes_folder(void);
void test_set_comm_mode(void);
void test_is_default_letter(void);
void test_default_letter_received(void);
void test_process_invalid_kings_letter(void);
void test_process_kp0(void);
void test_process_kp1(void);
void test_process_kp2(void);
void test_process_kp8(void);
void test_process_kp16(void);
void test_process_kp17(void);

// Helpers
ck_err_t set_action_mode(ck_action_mode_t mode);
ck_err_t set_city_mode(ck_city_mode_t mode);
void start_200ms_timer(void);

void check_kings_doc_folder(ck_folder_t *folder);
void check_mayors_doc_folder(ck_folder_t *folder);
void check_ck_folder(ck_folder_t *folder);

void setup_test(void);

// Functions for setting up the city data
void init_pages(void);
void init_docs(void);
void init_lists(void);
void init_folders(void);
void init_data(void);

int main(void) {
  test_process_before_init();
  test_mayor_init();
  test_process_invalid_kings_letter();
  test_process_kp0();
  test_process_kp1();
  test_process_kp2();
  test_process_kp8();
  test_process_kp16();
  test_process_kp17();
  test_add_mayors_page();
  test_send_document();
  test_send_page();
  test_send_mayors_page();
  test_is_kings_envelope();
  test_get_envelopes_folder();
  test_set_comm_mode();
  test_is_default_letter();
  test_default_letter_received();
}

void test_process_before_init(void) {
  ck_letter_t letter = {.envelope.envelope_no = 0};
  ASSERT(ck_process_kings_letter(&letter) == CK_ERR_NOT_INITIALIZED,
         "ck_process_kings_letter() shouldn't succeed.");
}

void test_mayor_init(void) {
  init_data();

  ck_id_t ck_id = {
      .city_address = 0,  // Illegal city address
      .base_no = test_base_no,
      .base_no_is_known = true,
      .base_no_has_extended_id = false,
  };

  // Set up with some illegal parameters first.
  ck_mayor_t mayor = {
      .ean_no = illegal_ean_no,  // Illegal EAN
      .serial_no = test_serial_no,
      .ck_id = ck_id,
      // Illegal function pointers
      .set_action_mode = NULL,
      .set_city_mode = NULL,
      .start_200ms_timer = start_200ms_timer,
      .folder_count = 0,  // Wrong folder count
      .folders = data.folders,
      .list_count = 0,  // Wrong list count
      .lists = data.lists,
  };

  // Test illegal parameters.
  ASSERT(ck_mayor_init(&mayor) != CK_OK, "illegal parameter should fail.");

  mayor.ean_no = test_ean_no;
  ASSERT(ck_mayor_init(&mayor) != CK_OK, "illegal parameter should fail.");

  mayor.ck_id.city_address = test_city_address;
  ASSERT(ck_mayor_init(&mayor) != CK_OK, "illegal parameter should fail.");

  mayor.set_action_mode = set_action_mode;
  ASSERT(ck_mayor_init(&mayor) != CK_OK, "illegal parameter should fail.");

  mayor.set_city_mode = set_city_mode;
  ASSERT(ck_mayor_init(&mayor) != CK_OK, "illegal parameter should fail.");

  mayor.folder_count = FOLDER_COUNT;
  ASSERT(ck_mayor_init(&mayor) != CK_OK, "illegal parameter should fail.");

  // Test correct parameters.
  mayor.list_count = LIST_COUNT;

  ASSERT(ck_mayor_init(&mayor) == CK_OK, "");
  check_kings_doc_folder(&data.folders[0]);
  check_mayors_doc_folder(&data.folders[1]);
}

void test_add_mayors_page(void) {
  setup_test();

  ASSERT(ck_add_mayors_page(NULL) != CK_OK, "add NULL page returned OK.");

  // Add pages until capacity is reached then check if error is returned.
  // Mayor's doc always contains two pages from the start.
  ck_page_t page = {.line_count = CK_MAX_LINES_PER_PAGE};
  for (int i = 0; i < CK_MAX_PAGES_PER_DOCUMENT - 2; i++) {
    ASSERT(ck_add_mayors_page(&page) == CK_OK,
           "add page failed on iteration %d.", i);
  }

  ASSERT(ck_add_mayors_page(&page) != CK_OK,
         "adding beyond capacity should fail.");
}

void test_send_document(void) {
  setup_test();

  ASSERT(ck_set_comm_mode(CK_COMM_MODE_COMMUNICATE) == CK_OK, "");

  ASSERT(ck_send_document(2) == CK_OK,
         "sending with valid folder number failed.");

  ASSERT(ck_send_document(FOLDER_COUNT + 1) != CK_OK,
         "invalid folder number should fail.");
}

void test_send_page(void) {
  setup_test();

  ASSERT(ck_set_comm_mode(CK_COMM_MODE_COMMUNICATE) == CK_OK, "");

  ASSERT(ck_send_page(2, 0) == CK_OK,
         "sending with valid folder and page numbers failed.");

  ASSERT(ck_send_page(FOLDER_COUNT + 1, 0) != CK_OK,
         "invalid folder number should fail.");

  ASSERT(ck_send_page(2, 1) != CK_OK, "invalid page number should fail.");
}

void test_send_mayors_page(void) {
  setup_test();

  ASSERT(ck_set_comm_mode(CK_COMM_MODE_COMMUNICATE) == CK_OK, "");

  ASSERT(ck_send_mayors_page(0) == CK_OK,
         "sending with valid mayor's page number failed.");

  ASSERT(ck_send_mayors_page(2) != CK_OK, "invalid page number should fail.");
}

void test_is_kings_envelope(void) {
  ck_envelope_t good_envelope = {.envelope_no = 0};
  ck_envelope_t bad_envelope = {.envelope_no = 1};

  ASSERT(ck_is_kings_envelope(&good_envelope) == CK_OK, "");
  ASSERT(ck_is_kings_envelope(&bad_envelope) != CK_OK, "");
}

// First, check if envelope 200 is assigned to folder 2. Then, assign it, and
// test again.
void test_get_envelopes_folder(void) {
  setup_test();

  const ck_envelope_t envelope = {.envelope_no = 200};

  ck_folder_t *folder = NULL;

  ASSERT(ck_get_envelopes_folder(&envelope, &folder) == CK_ERR_FALSE,
         "checking for non-existent envelope did not return false.");

  ck_kp2_args_t args = {
      .folder_no = 2,
      .envelope = envelope,
      .envelope_action = CK_ENVELOPE_ASSIGN,
  };
  ck_letter_t letter;

  ASSERT(ck_create_kings_page_2(&args, &letter.page) == CK_OK, "");
  ASSERT(ck_process_kings_letter(&letter) == CK_OK, "");
  ASSERT(ck_get_envelopes_folder(&envelope, &folder) == CK_OK, "");
  ASSERT(folder == &data.folders[2],
         "wrong folder returned, expected: %u, got: %u",
         data.folders[2].folder_no, folder->folder_no);
}

void test_set_comm_mode(void) {
  setup_test();

  ASSERT(ck_get_comm_mode() == CK_COMM_MODE_SILENT,
         "comm mode not init to CK_COMM_MODE_SILENT.");
  ASSERT(ck_set_comm_mode(CK_COMM_MODE_COMMUNICATE) == CK_OK, "");
  ck_comm_mode_t mode = ck_get_comm_mode();

  ASSERT(mode == CK_COMM_MODE_COMMUNICATE,
         "ck_get_comm_mode() returned wrong mode, expected: %u, got: %u.",
         CK_COMM_MODE_COMMUNICATE, mode);
}

void test_is_default_letter(void) {
  ck_letter_t dletter = ck_default_letter();
  ASSERT(ck_is_default_letter(&dletter) == CK_OK, "");

  ck_letter_t illegal_letter = {.envelope.envelope_no = 0};
  ASSERT(ck_is_default_letter(&illegal_letter) != CK_OK, "");
}

void test_default_letter_received(void) {
  setup_test();

  ASSERT(ck_default_letter_received() == CK_OK, "");

  ck_comm_mode_t mode = ck_get_comm_mode();
  ASSERT(mode == CK_COMM_MODE_LISTEN_ONLY,
         "comm mode not correct after default letter reception, expected: %u, "
         "got: %u.",
         CK_COMM_MODE_COMMUNICATE, mode);

  // Timeout before default letter is received
  setup_test();

  ASSERT(ck_default_letter_timeout() == CK_OK, "");
  ASSERT(ck_default_letter_received() == CK_OK, "");
  ASSERT(ck_get_comm_mode() == CK_COMM_MODE_SILENT,
         "comm mode changed even though default letter was received after "
         "timeout.");
}

void test_process_invalid_kings_letter(void) {
  setup_test();

  // Begin with invalid letters
  ck_letter_t empty_letter;
  memset(&empty_letter, 0, sizeof(ck_letter_t));

  ASSERT(ck_process_kings_letter(&empty_letter) != CK_OK,
         "empty_letter should fail.");

  ck_letter_t invalid_letter1 = {
      .page = {.line_count = 1},
  };

  ASSERT(ck_process_kings_letter(&invalid_letter1) != CK_OK,
         "invalid letter should fail.");

  ck_letter_t invalid_letter2 = {
      .envelope.is_remote = true,
      .page = {.line_count = CK_MAX_LINES_PER_PAGE},
  };

  ASSERT(ck_process_kings_letter(&invalid_letter2) != CK_OK,
         "invalid letter should fail.");
}

void test_process_kp0(void) {
  setup_test();

  ck_kp0_args_t args = {
      .address = test_city_address,
      .action_mode = CK_ACTION_MODE_FREEZE,
      .comm_mode = CK_COMM_MODE_COMMUNICATE,
      .comm_flags = CK_COMM_RESET,
      .city_mode = CK_CITY_MODE_KEEP_CURRENT,
  };

  ck_letter_t letter;
  memset(&letter.envelope, 0, sizeof(letter.envelope));

  ASSERT(ck_create_kings_page_0(&args, &letter.page) == CK_OK, "");

  ASSERT(ck_process_kings_letter(&letter) == CK_OK, "");

  ck_comm_mode_t comm_mode = ck_get_comm_mode();
  ASSERT(comm_mode == args.comm_mode, "");
}

void test_process_kp1(void) {
  setup_test();

  ck_kp1_args_t args = {
      .address = test_city_address,
      .base_no = test_base_no,
      .mayor_response_no = CK_NO_RESPONSE_REQUESTED,
  };

  ck_letter_t letter;
  memset(&letter.envelope, 0, sizeof(letter.envelope));

  ASSERT(ck_create_kings_page_1(&args, &letter.page) == CK_OK, "");

  ASSERT(ck_process_kings_letter(&letter) == CK_OK, "");

  // Invalid base no. (base no + city address will be too big)
  uint32_t invalid_base_no = illegal_can_std_id - 1;
  args.base_no = invalid_base_no;
  ASSERT(ck_create_kings_page_1(&args, &letter.page) == CK_OK, "");

  ASSERT(ck_process_kings_letter(&letter) != CK_OK,
         "invalid base no page should fail.");

  // Invalid base no (extended ID)
  invalid_base_no = illegal_can_ext_id - 1;
  args.base_no = invalid_base_no;
  args.has_extended_id = true;
  ASSERT(ck_create_kings_page_1(&args, &letter.page) == CK_OK, "");

  ASSERT(ck_process_kings_letter(&letter) != CK_OK,
         "invalid extended base no page should fail.");

  // Invalid mayor response page
  args.base_no = test_base_no;
  args.mayor_response_no = 3;
  ASSERT(ck_create_kings_page_1(&args, &letter.page) == CK_OK, "");

  ASSERT(ck_set_comm_mode(CK_COMM_MODE_COMMUNICATE) == CK_OK, "");

  ASSERT(ck_process_kings_letter(&letter) != CK_OK,
         "invalid mayor's respnse page should fail.");
}

// First assign envelopes 200 and 300 to folder 2, then transfer envelope 300 to
// folder 3, then expel envelope 200, then disable envelope 300.
void test_process_kp2(void) {
  setup_test();

  // Assign envelope no 200 to folder no 2.
  ck_kp2_args_t args = {
      .address = 0,
      .folder_no = 2,
      .envelope.envelope_no = 200,  // NOLINT
      .envelope_action = CK_ENVELOPE_ASSIGN,
  };

  ck_letter_t letter;
  memset(&letter.envelope, 0, sizeof(letter.envelope));

  ASSERT(ck_create_kings_page_2(&args, &letter.page) == CK_OK, "");
  ASSERT(ck_process_kings_letter(&letter) == CK_OK, "");

  uint32_t got_envelope_no = data.folders[2].envelopes[0].envelope_no;
  ASSERT(got_envelope_no == args.envelope.envelope_no,
         "assign: wrong envelope no, expected: %u, got: %u.",
         args.envelope.envelope_no, got_envelope_no);

  ASSERT(data.folders[2].envelope_count == 1,
         "assign: wrong envelope count for folder 2, expected: 1, got: %u.",
         data.folders[2].envelope_count);

  // Assign envelope no 300 to folder no 2.
  args.folder_no = 2;
  args.envelope.envelope_no = 300;  // NOLINT
  ASSERT(ck_create_kings_page_2(&args, &letter.page) == CK_OK, "");

  ASSERT(ck_process_kings_letter(&letter) == CK_OK, "");

  got_envelope_no = data.folders[2].envelopes[1].envelope_no;
  ASSERT(got_envelope_no == args.envelope.envelope_no,
         "assign: wrong envelope no, expected: %u, got: %u.",
         args.envelope.envelope_no, got_envelope_no);

  ASSERT(data.folders[2].envelope_count == 2,
         "assign: wrong envelope count for folder 2, expected: 2, got: %u.",
         data.folders[2].envelope_count);

  // Transfer envelope no 300 to folder no 3, and enable it.
  args.folder_no = 3;
  args.envelope_action = CK_ENVELOPE_TRANSFER;
  args.envelope.envelope_no = 300;  // NOLINT
  args.envelope.enable = true;
  ASSERT(ck_create_kings_page_2(&args, &letter.page) == CK_OK, "");
  ASSERT(ck_process_kings_letter(&letter) == CK_OK, "");

  got_envelope_no = data.folders[3].envelopes[0].envelope_no;
  ASSERT(got_envelope_no == args.envelope.envelope_no,
         "transfer: wrong envelope no, expected: %u, got: %u.",
         args.envelope.envelope_no, got_envelope_no);

  ASSERT(data.folders[2].envelope_count == 1,
         "transfer: wrong envelope count for folder 2, expected: 1, got: %u.",
         data.folders[2].envelope_count);

  ASSERT(data.folders[3].envelope_count == 1,
         "transfer: wrong envelope count for folder 3, expected: 1, got: %u.",
         data.folders[3].envelope_count);

  ASSERT(data.folders[3].envelopes[0].enable,
         "transfer: envelope 300 not enabled.");

  // Expel envelope no 200.
  args.envelope.envelope_no = 200;  // NOLINT
  args.envelope_action = CK_ENVELOPE_EXPEL;
  args.envelope.enable = false;
  ASSERT(ck_create_kings_page_2(&args, &letter.page) == CK_OK, "");
  ASSERT(ck_process_kings_letter(&letter) == CK_OK, "");

  ASSERT(data.folders[2].envelope_count == 0,
         "expel: wrong envelope count for folder 2, expected: 0, got: %u.",
         data.folders[2].envelope_count);

  // Disable envelope no 300.
  args.envelope.envelope_no = 300;  // NOLINT
  args.envelope_action = CK_ENVELOPE_NO_ACTION;
  args.envelope.enable = false;
  ASSERT(ck_create_kings_page_2(&args, &letter.page) == CK_OK, "");
  ASSERT(ck_process_kings_letter(&letter) == CK_OK, "");

  ASSERT(!data.folders[3].envelopes[0].enable,
         "disable: envelope 300 enabled after disable command.");
}

void test_process_kp8(void) {
  // This sets up the mayor with the default bit timing.
  setup_test();

  const ck_can_bit_timing_t next_bit_timing = {
      .prescaler = 10,
      .time_quanta = 10,
      .phase_seg2 = 4,
      .sjw = 2,
  };

  ck_letter_t letter;
  memset(&letter.envelope, 0, sizeof(letter.envelope));

  ASSERT(ck_create_kings_page_8(test_city_address, &next_bit_timing,
                                &letter.page) == CK_OK,
         "");
  ASSERT(ck_process_kings_letter(&letter) == CK_OK, "");

  // Send kp0 reset comm message
  ck_kp0_args_t args = {
      .address = test_city_address,
      .action_mode = CK_ACTION_MODE_KEEP_CURRENT,
      .comm_mode = CK_COMM_MODE_KEEP_CURRENT,
      .comm_flags = CK_COMM_RESET,
      .city_mode = CK_CITY_MODE_KEEP_CURRENT,
  };

  ASSERT(ck_create_kings_page_0(&args, &letter.page) == CK_OK, "");
  ASSERT(ck_process_kings_letter(&letter) == CK_OK, "");

  // Emulate reception of a letter, so the bit timing is saved persistently.
  ASSERT(ck_correct_letter_received() == CK_OK, "");

  ck_can_bit_timing_t current_bit_timing;
  ASSERT(ck_load_bit_timing(&current_bit_timing) == CK_OK, "");

  ASSERT(memcmp(&current_bit_timing, &next_bit_timing,
                sizeof(ck_can_bit_timing_t)) == 0,
         "bit timing was not set correctly."
         "expected: prescaler: %u, tq: %u, ps2: %u, sjw: %u."
         "got: prescaler: %u, tq: %u, ps2: %u, sjw: %u.",
         next_bit_timing.prescaler, next_bit_timing.time_quanta,
         next_bit_timing.phase_seg2, next_bit_timing.sjw,
         current_bit_timing.prescaler, current_bit_timing.time_quanta,
         current_bit_timing.phase_seg2, current_bit_timing.sjw);
}

// Moves the document in folder 2 to folder 3.
void test_process_kp16(void) {
  // Places the tx doc in folder 2 and rx doc in folder 3.
  setup_test();

  // Don't use ck_create_kings_page_16() here since it performs error checks  on
  // the args and we want to use illegal args later.

  // NOLINTBEGIN(*-magic-numbers)
  // Insert document T0.1 into folder 3.
  ck_letter_t letter;
  memset(&letter.envelope, 0, sizeof(letter.envelope));

  letter.page.line_count = CK_MAX_LINES_PER_PAGE;
  letter.page.lines[0] = test_city_address;
  letter.page.lines[1] = CK_KP16;
  uint8_t *folder_no = &letter.page.lines[2];
  *folder_no = 3;

  uint8_t dlc = 4;
  letter.page.lines[3] = dlc | (1 << 7);  // MSB should be 1.

  // MSB should be 1.
  letter.page.lines[4] =
      CK_DIRECTION_TRANSMIT | CK_DOCUMENT_INSERT << 4 | (1 << 7);

  uint8_t list_no = 0;
  uint8_t doc_no = 1;
  letter.page.lines[5] = list_no;
  letter.page.lines[6] = doc_no;
  // NOLINTEND(*-magic-numbers)

  ASSERT(ck_process_kings_letter(&letter) == CK_OK, "");
  ASSERT(data.folders[3].dlc == dlc,
         "insert: wrong DLC, expected: %u, got: %u.", dlc, data.folders[3].dlc);

  ASSERT(data.folders[3].direction == CK_DIRECTION_TRANSMIT,
         "insert: wrong direction, expected: %u, got: %u.",
         CK_DIRECTION_TRANSMIT, data.folders[3].direction);

  ASSERT(!data.folders[3].enable, "insert: folder should be disabled.");

  ASSERT(data.folders[3].doc_list_no == list_no,
         "insert: wrong list no, expected: %u, got: %u.", list_no,
         data.folders[3].doc_list_no);

  ASSERT(data.folders[3].doc_no == doc_no,
         "insert: wrong doc no, expected: %u, got: %u.", doc_no,
         data.folders[3].doc_no);

  // Remove document from folder 2
  *folder_no = 2;
  ASSERT(ck_process_kings_letter(&letter) == CK_OK, "");
  ASSERT(!data.folders[2].enable, "remove: folder should be disabled.");

  // Test illegal parameters
  *folder_no = 0;
  ASSERT(ck_process_kings_letter(&letter) != CK_OK,
         "illegal folder should fail.");

  *folder_no = FOLDER_COUNT + 1;
  ASSERT(ck_process_kings_letter(&letter) != CK_OK,
         "non-existent folder should fail.");
}

// Test creating a line, creating a page, and creating a document.
void test_process_kp17(void) {
  setup_test();

  // Don't use ck_create_kings_page_17() here since it performs error checks  on
  // the args and we want to use illegal args later.

  ck_letter_t letter;
  memset(&letter.envelope, 0, sizeof(letter.envelope));

  letter.page.line_count = CK_MAX_LINES_PER_PAGE;
  letter.page.lines[0] = test_city_address;
  letter.page.lines[1] = CK_KP17;
  // Try some illegal parameters
  letter.page.lines[2] = CK_LIST_BIT << 3;

  const uint8_t illegal_list_number = 8;
  const uint8_t illegal_record_number = 8;
  const uint8_t illegal_target_position = 8;

  // NOLINTBEGIN(*-magic-numbers)
  uint8_t *source_list_number = &letter.page.lines[3];
  uint8_t *source_record_number = &letter.page.lines[4];
  letter.page.lines[5] = 0;  // Target list no, will be the same for all
  uint8_t *target_record_number = &letter.page.lines[6];
  uint8_t *target_position = &letter.page.lines[7];
  // NOLINTEND(*-magic-numbers)

  *source_list_number = illegal_list_number;
  *source_record_number = 0;
  *target_record_number = illegal_record_number;
  *target_position = illegal_target_position;
  ASSERT(ck_process_kings_letter(&letter) != CK_OK, "");

  letter.page.lines[2] = CK_LIST_LINE << 3;
  ASSERT(ck_process_kings_letter(&letter) != CK_OK, "");

  *source_list_number = 0;
  ASSERT(ck_process_kings_letter(&letter) != CK_OK, "");

  *target_record_number = 0;
  ASSERT(ck_process_kings_letter(&letter) != CK_OK, "");

  // Create line R0.0 using Bit R0.0. Should have value 1 (0b01).
  *target_position = 0;
  ASSERT(ck_process_kings_letter(&letter) == CK_OK, "");

  // Create line R0.1 using Bit R0.0. Should have value 2 (0b10).
  *target_record_number = 1;
  *target_position = 1;
  ASSERT(ck_process_kings_letter(&letter) == CK_OK, "");

  // Verify the data
  uint8_t *line_r00 = (uint8_t *)data.rx_line_list->records[0];
  uint8_t *line_r01 = (uint8_t *)data.rx_line_list->records[1];
  ASSERT(*line_r00 == 1 && *line_r01 == 2,
         "wrong line data, expected: 1, 2, got: %u, %u.", *line_r00, *line_r01);

  // Create page R0.1 with lines R0.0 and R0.1
  letter.page.lines[2] = CK_LIST_PAGE << 3;
  *target_record_number = 1;
  *source_record_number = 0;
  *target_position = 0;
  ASSERT(ck_process_kings_letter(&letter) == CK_OK, "");

  *source_record_number = 1;
  *target_position = 1;
  ASSERT(ck_process_kings_letter(&letter) == CK_OK, "");

  // Verify the data
  ck_page_t *page_r01 = (ck_page_t *)data.rx_page_list->records[1];
  ASSERT(page_r01->line_count == 2, "wrong line count, expected: 2, got: %u.",
         page_r01->line_count);

  ASSERT(page_r01->lines[0] == 1 && page_r01->lines[1] == 2,
         "wrong page data, expected: 1, 2, got: %u, %u.", page_r01->lines[0],
         page_r01->lines[1]);

  // Create document R0.1 which contains two copies of page R0.1.
  letter.page.lines[2] = CK_LIST_DOCUMENT << 3;
  *target_record_number = 1;
  *source_record_number = 1;
  *target_position = 0;
  ASSERT(ck_process_kings_letter(&letter) == CK_OK, "");

  *target_position = 1;
  ASSERT(ck_process_kings_letter(&letter) == CK_OK, "");

  // Verify the data
  ck_document_t *doc_r01 = (ck_document_t *)data.rx_doc_list->records[1];
  ASSERT(doc_r01->page_count == 2, "wrong page count, expected: 2, got: %u.",
         doc_r01->page_count);

  ASSERT(doc_r01->pages[0]->lines[0] == 1 && doc_r01->pages[0]->lines[1] == 2 &&
             doc_r01->pages[1]->lines[0] == 1 &&
             doc_r01->pages[1]->lines[1] == 2,
         "wrong page data, expected: 1, 2, 1, 2, "
         "got: %u, %u, %u, %u.",
         doc_r01->pages[0]->lines[0], doc_r01->pages[0]->lines[1],
         doc_r01->pages[1]->lines[0], doc_r01->pages[1]->lines[1]);
}

ck_err_t set_action_mode(ck_action_mode_t mode) {
  (void)mode;
  return CK_OK;
}

ck_err_t set_city_mode(ck_city_mode_t mode) {
  (void)mode;
  return CK_OK;
}

void start_200ms_timer(void) {
}

void check_kings_doc_folder(ck_folder_t *folder) {
  check_ck_folder(folder);

  ASSERT(folder->envelopes[0].envelope_no == 0,
         "wrong envelope number, expected: 0, got: %u.",
         folder->envelopes[0].envelope_no);

  ASSERT(folder->direction == CK_DIRECTION_RECEIVE,
         "wrong direction, expected: %u, got: %u.", CK_DIRECTION_RECEIVE,
         folder->direction);
}

void check_mayors_doc_folder(ck_folder_t *folder) {
  check_ck_folder(folder);

  ASSERT(folder->envelopes[0].envelope_no == test_base_no + test_city_address,
         "wrong envelope number, expected: %u, got: %u.",
         test_base_no + test_city_address, folder->envelopes[0].envelope_no);

  ASSERT(folder->direction == CK_DIRECTION_TRANSMIT,
         "wrong direction, expected: %u, got: %u.", CK_DIRECTION_TRANSMIT,
         folder->direction);
}

void check_ck_folder(ck_folder_t *folder) {
  ASSERT(folder->enable, "folder not enabled.");
  ASSERT(folder->envelopes[0].enable, "envelope not enabled.");

  ASSERT(folder->envelope_count == 1,
         "wrong envelope count, expected: 1, got: %u.", folder->envelope_count);

  ASSERT(folder->dlc == CK_CAN_MAX_DLC, "wrong DLC, expected: %u, got: %u.",
         CK_CAN_MAX_DLC, folder->dlc);
}

void setup_test(void) {
  init_data();

  ck_id_t ck_id = {
      .city_address = test_city_address,
  };

  ck_mayor_t mayor = {
      .ean_no = test_ean_no,
      .serial_no = test_ean_no,
      .ck_id = ck_id,
      .set_action_mode = set_action_mode,
      .set_city_mode = set_city_mode,
      .start_200ms_timer = start_200ms_timer,
      .folder_count = FOLDER_COUNT,
      .folders = data.folders,
      .list_count = LIST_COUNT,
      .lists = data.lists,
  };

  if (ck_mayor_init(&mayor) != CK_OK) {
    exit(TEST_SETUP_FAIL);
  }
}

void init_data(void) {
  memset(&data, 0, sizeof(data));
  data.rx_bits[0] = 1;
  init_pages();
  init_docs();
  init_lists();
  init_folders();
}

void init_pages(void) {
  data.rx_pages[0].line_count = CK_MAX_LINES_PER_PAGE;
  const uint8_t rx_line[8] = {7, 6, 5, 4, 3, 2, 1, 0};
  memcpy(data.rx_pages[0].lines, rx_line, sizeof(rx_line));

  data.tx_pages[0].line_count = 0;
}

void init_docs(void) {
  data.rx_docs[0].direction = CK_DIRECTION_RECEIVE;
  data.rx_docs[0].page_count = 1;
  data.rx_docs[0].pages[0] = data.rx_pages;

  data.tx_docs[0].direction = CK_DIRECTION_TRANSMIT;
  data.tx_docs[0].page_count = 1;
  data.tx_docs[0].pages[0] = data.tx_pages;
}

void init_lists(void) {
  // Transmit doc list number 0. Required by the mayor lib, to create the
  // mayor's document. We also add a test document for testing
  // ck_send_document().
  data.tx_doc_list = &data.lists[0];
  data.tx_doc_list->type = CK_LIST_DOCUMENT;
  data.tx_doc_list->direction = CK_DIRECTION_TRANSMIT;
  data.tx_doc_list->list_no = 0;
  data.tx_doc_list->record_count = TX_DOCUMENT_COUNT;
  // Record no 0 reserved for the mayor's document.
  data.tx_doc_list->records[1] = &data.tx_docs[0];

  // Receive doc list number 0
  data.rx_doc_list = &data.lists[1];
  data.rx_doc_list->type = CK_LIST_DOCUMENT;
  data.rx_doc_list->direction = CK_DIRECTION_RECEIVE;
  data.rx_doc_list->list_no = 0;
  data.rx_doc_list->record_count = RX_DOCUMENT_COUNT;
  for (int i = 0; i < RX_DOCUMENT_COUNT; i++) {
    data.rx_doc_list->records[i] = &data.rx_docs[i];
  }

  // Predefined bit list
  data.rx_bit_list = &data.lists[2];
  data.rx_bit_list->type = CK_LIST_BIT;
  data.rx_bit_list->direction = CK_DIRECTION_RECEIVE;
  data.rx_bit_list->list_no = 0;
  data.rx_bit_list->record_count = RX_BIT_COUNT;
  data.rx_bit_list->records[0] = data.rx_bits;

  // Predefined line list
  data.rx_line_list = &data.lists[3];
  data.rx_line_list->type = CK_LIST_LINE;
  data.rx_line_list->direction = CK_DIRECTION_RECEIVE;
  data.rx_line_list->list_no = 0;
  data.rx_line_list->record_count = RX_LINE_COUNT;
  for (int i = 0; i < RX_LINE_COUNT; i++) {
    data.rx_line_list->records[i] = &data.rx_lines[i];
  }

  // Predrx_line_list->ge list.
  data.rx_page_list = &data.lists[4];
  data.rx_page_list->type = CK_LIST_PAGE;
  data.rx_page_list->direction = CK_DIRECTION_RECEIVE;
  data.rx_page_list->list_no = 0;
  data.rx_page_list->record_count = RX_PAGE_COUNT;
  for (int i = 0; i < RX_PAGE_COUNT; i++) {
    data.rx_page_list->records[i] = &data.rx_pages[i];
  }
}

void init_folders(void) {
  // Set up folders 2 and 3. 0 and 1 are reserved for the king's doc and
  // mayor's doc, which are set up by the mayor lib.
  data.folders[2].folder_no = 2;
  data.folders[2].doc_list_no = 0;
  data.folders[2].doc_no = 1;  // Doc no 0 is reserved for the mayor's doc
  data.folders[2].direction = CK_DIRECTION_TRANSMIT;
  data.folders[2].dlc = CK_CAN_MAX_DLC;
  data.folders[2].has_rtr = false;
  data.folders[2].enable = true;
  data.folders[2].envelope_count = 0;  // This is set by the king
  memset(data.folders[2].envelopes, 0,
         sizeof(ck_envelope_t) * CK_MAX_ENVELOPES_PER_FOLDER);

  data.folders[3].folder_no = 3;
  data.folders[3].doc_list_no = 0;
  data.folders[3].doc_no = 0;
  data.folders[3].direction = CK_DIRECTION_RECEIVE;
  data.folders[3].dlc = CK_CAN_MAX_DLC;
  data.folders[3].has_rtr = false;
  data.folders[3].enable = true;
  data.folders[3].envelope_count = 0;
  memset(data.folders[3].envelopes, 0,
         sizeof(ck_envelope_t) * CK_MAX_ENVELOPES_PER_FOLDER);
}
