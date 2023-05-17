#include "mayor.h"

#include <stdio.h>
#include <string.h>

#include "test.h"

#define RX_BIT_COUNT 1
#define RX_LINE_COUNT 2
#define RX_PAGE_COUNT 2
#define TX_PAGE_COUNT 2  // Mayor's pages
#define RX_DOCUMENT_COUNT 2
#define TX_DOCUMENT_COUNT 1  // For the mayor's document
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

static test_err_t test_mayor_init(void);
static test_err_t test_process_kings_letter(void);
static test_err_t test_add_mayors_page(void);
static test_err_t test_ck_send_document(void);
static test_err_t test_ck_send_mayors_page(void);
static test_err_t test_process_kp0(void);
static test_err_t test_process_kp1(void);
static test_err_t test_process_kp2(void);
static test_err_t test_process_kp16(void);
static test_err_t test_process_kp17(void);

// Helpers
static ck_err_t set_action_mode(ck_action_mode_t mode);
static ck_err_t set_comm_mode(ck_comm_mode_t mode);
static ck_err_t set_city_mode(ck_city_mode_t mode);

static test_err_t check_kings_doc_folder(ck_folder_t *folder);
static test_err_t check_mayors_doc_folder(ck_folder_t *folder);

// Functions for setting up the city data
static void init_pages(void);
static void init_docs(void);
static void init_lists(void);
static void init_folders(void);
static void init_data(void);

static test_err_t setup_test(void);  // For setting up the test harness.

int main(void) {
  // Test process before init
  ck_letter_t letter = {.envelope.envelope_no = 0};
  if (ck_process_kings_letter(&letter) != CK_ERR_NOT_INITIALIZED) {
    printf("Process before init succeeded.\n");
    return TEST_FAIL;
  }
  if (test_mayor_init() != TEST_PASS) {
    return TEST_FAIL;
  }
  if (test_process_kings_letter() != TEST_PASS) {
    return TEST_FAIL;
  }
  if (test_add_mayors_page() != TEST_PASS) {
    return TEST_FAIL;
  }
  if (test_ck_send_document() != TEST_PASS) {
    return TEST_FAIL;
  }
  return test_ck_send_mayors_page();
}

static test_err_t test_mayor_init(void) {
  init_data();

  // Set up with some illegal parameters first.
  ck_mayor_t mayor = {
      .ean_no = illegal_ean_no,  // Illegal EAN
      .serial_no = test_serial_no,
      .city_address = 0,  // illegal city address
      .base_no = test_base_no,
      .has_extended_id = false,
      // Illegal function pointers
      .set_action_mode = NULL,
      .set_comm_mode = NULL,
      .set_city_mode = NULL,
      .folder_count = 0,  // Wrong folder count
      .folders = data.folders,
      .list_count = 0,  // Wrong list count
      .lists = data.lists,
  };

  // Test illegal parameters.
  if (ck_mayor_init(&mayor) == CK_OK) {
    printf("mayor_init: illegal parameter returned OK.\n");
    return TEST_FAIL;
  }

  mayor.ean_no = test_ean_no;
  if (ck_mayor_init(&mayor) == CK_OK) {
    printf("mayor_init: illegal parameter returned OK.\n");
    return TEST_FAIL;
  }

  mayor.city_address = test_city_address;
  if (ck_mayor_init(&mayor) == CK_OK) {
    printf("mayor_init: illegal parameter returned OK.\n");
    return TEST_FAIL;
  }

  mayor.set_action_mode = set_action_mode;
  if (ck_mayor_init(&mayor) == CK_OK) {
    printf("mayor_init: illegal parameter returned OK.\n");
    return TEST_FAIL;
  }

  mayor.set_comm_mode = set_comm_mode;
  if (ck_mayor_init(&mayor) == CK_OK) {
    printf("mayor_init: illegal parameter returned OK.\n");
    return TEST_FAIL;
  }

  mayor.set_city_mode = set_city_mode;
  if (ck_mayor_init(&mayor) == CK_OK) {
    printf("mayor_init: illegal parameter returned OK.\n");
    return TEST_FAIL;
  }

  mayor.folder_count = FOLDER_COUNT;
  if (ck_mayor_init(&mayor) == CK_OK) {
    printf("mayor_init: illegal parameter returned OK.\n");
    return TEST_FAIL;
  }

  // Test correct parameters.
  mayor.list_count = LIST_COUNT;
  if (ck_mayor_init(&mayor) != CK_OK) {
    printf("mayor_init: failed to init mayor.\n");
    return TEST_FAIL;
  }

  // Check if folders 0 and 1 have been populated correctly.
  if (check_kings_doc_folder(&data.folders[0]) != TEST_PASS) {
    printf("mayor_init: incorrect king's folder.\n");
    return TEST_FAIL;
  }

  if (check_mayors_doc_folder(&data.folders[1]) != TEST_PASS) {
    printf("mayor_init: incorrect mayor's folder.\n");
    return TEST_FAIL;
  }

  return TEST_PASS;
}

static test_err_t test_process_kings_letter(void) {
  test_err_t err = setup_test();
  if (err != TEST_PASS) {
    return err;
  }

  // Begin with invalid letters
  ck_letter_t empty_letter;
  memset(&empty_letter, 0, sizeof(ck_letter_t));
  if (ck_process_kings_letter(&empty_letter) == CK_OK) {
    printf("process_kings_letter: empty_letter returned OK.\n");
    return TEST_FAIL;
  }

  ck_letter_t invalid_letter1 = {
      .page = {.line_count = 1},
  };
  if (ck_process_kings_letter(&invalid_letter1) == CK_OK) {
    printf("process_kings_letter: invalid letter 1 returned OK.\n");
    return TEST_FAIL;
  }

  ck_letter_t invalid_letter2 = {
      .envelope.is_remote = true,
      .page = {.line_count = CK_MAX_LINES_PER_PAGE},
  };
  if (ck_process_kings_letter(&invalid_letter2) == CK_OK) {
    printf("process_kings_letter: invalid letter 2 returned OK.\n");
    return TEST_FAIL;
  }

  // Test processing various pages
  if (test_process_kp0() != TEST_PASS) {
    return TEST_FAIL;
  }
  if (test_process_kp1() != TEST_PASS) {
    return TEST_FAIL;
  }
  if (test_process_kp2() != TEST_PASS) {
    return TEST_FAIL;
  }
  if (test_process_kp16() != TEST_PASS) {
    return TEST_FAIL;
  }
  if (test_process_kp17() != TEST_PASS) {
    return TEST_FAIL;
  }
  return TEST_PASS;
}

static test_err_t test_add_mayors_page(void) {
  test_err_t err = setup_test();
  if (err != TEST_PASS) {
    return err;
  }
  ck_err_t ret = ck_add_mayors_page(NULL);
  if (ret == CK_OK) {
    printf("add_mayors_page: add NULL page returned OK.\n");
    return TEST_FAIL;
  }

  // Add pages until capacity is reached then check if error is returned.
  // Mayor's doc always contains two pages from the start.
  ck_page_t page = {.line_count = CK_MAX_LINES_PER_PAGE};
  for (int i = 0; i < CK_MAX_PAGES_PER_DOCUMENT - 2; i++) {
    if (ck_add_mayors_page(&page) != CK_OK) {
      printf("add_mayors_page: add page failed on iteration %d.\n", i);
      return TEST_FAIL;
    }
  }

  // Adding beyond capacity should fail.
  if (ck_add_mayors_page(&page) == CK_OK) {
    printf("add_mayors_page: adding too many pages returned OK.\n");
    return TEST_FAIL;
  }

  return TEST_PASS;
}

static test_err_t test_ck_send_document(void) {
  test_err_t err = setup_test();
  if (err != TEST_PASS) {
    return err;
  }

  // Test valid folder number
  if (ck_send_document(2) != CK_OK) {
    printf("send_document: sending document failed.\n");
    return TEST_FAIL;
  }

  // Test invalid folder number
  if (ck_send_document(FOLDER_COUNT + 1) == CK_OK) {
    printf("send_document: invalid folder number returned OK.\n");
    return TEST_FAIL;
  }

  return TEST_PASS;
}

static test_err_t test_ck_send_mayors_page(void) {
  test_err_t err = setup_test();
  if (err != TEST_PASS) {
    return err;
  }

  // Test valid page number
  if (ck_send_mayors_page(0) != CK_OK) {
    printf("send_mayors_page: sending page failed.\n");
    return TEST_FAIL;
  }

  // Test invalid page number
  if (ck_send_mayors_page(2) == CK_OK) {
    printf("send_mayors_page: invalid page number returned OK.\n");
    return TEST_FAIL;
  }

  return TEST_PASS;
}

static test_err_t test_process_kp0(void) {
  ck_letter_t letter = {.page = {.line_count = CK_MAX_LINES_PER_PAGE}};
  letter.page.lines[0] = test_city_address;
  letter.page.lines[1] = CK_KP0;
  letter.page.lines[2] = CK_ACTION_MODE_FREEZE;
  letter.page.lines[3] = CK_COMM_MODE_SILENT;
  letter.page.lines[4] = CK_CITY_MODE_KEEP_CURRENT;

  if (ck_process_kings_letter(&letter) != CK_OK) {
    printf("process_kp0: failed to process page.\n");
    return TEST_FAIL;
  }
  return TEST_PASS;
}

static test_err_t test_process_kp1(void) {
  ck_letter_t letter;
  letter.page.lines[0] = test_city_address;
  letter.page.lines[1] = CK_KP1;
  letter.page.lines[2] = CK_NO_RESPONSE_REQUESTED;
  memcpy(&letter.page.lines[4], &test_base_no, sizeof(uint32_t));
  if (ck_process_kings_letter(&letter) != CK_OK) {
    printf("process_kp1: failed to process page.\n");
    return TEST_FAIL;
  }

  // Invalid base no. (base no + city address will be too big)
  uint32_t invalid_base_no = illegal_can_std_id - 1;
  memcpy(&letter.page.lines[4], &invalid_base_no, sizeof(invalid_base_no));
  if (ck_process_kings_letter(&letter) == CK_OK) {
    printf("process_kp1: invalid base no page returned OK.\n");
    return TEST_FAIL;
  }

  // Invalid base no (extended ID)
  invalid_base_no = illegal_can_ext_id - 1;
  memcpy(&letter.page.lines[4], &invalid_base_no, sizeof(invalid_base_no));
  letter.page.lines[7] |= 0x80;  // Extended ID flag  // NOLINT(*-magic-numbers)
  if (ck_process_kings_letter(&letter) == CK_OK) {
    printf("process_kp1: invalid ext base no page returned OK.\n");
    return TEST_FAIL;
  }

  // Invalid mayor response page
  letter.page.lines[2] = 3;
  if (ck_process_kings_letter(&letter) == CK_OK) {
    printf("process_kp1: invalid mayor's response page returned OK.\n");
    return TEST_FAIL;
  }

  return TEST_PASS;
}

// First assign envelopes 2 and 3 to folder 2, then transfer envelope 3 to
// folder 3, then expel envelope 2.
static test_err_t test_process_kp2(void) {
  test_err_t err = setup_test();
  if (err != TEST_PASS) {
    return err;
  }

  // Assign envelope no 2 to folder no 2.
  ck_letter_t letter;
  letter.page.lines[0] = test_city_address;
  letter.page.lines[1] = CK_KP2;

  uint32_t envelope2 = 2;
  uint32_t folder2 = 2;

  memcpy(&letter.page.lines[2], &envelope2, sizeof(envelope2));
  // NOLINTBEGIN(*-magic-numbers)
  letter.page.lines[6] = folder2;
  letter.page.lines[7] = CK_ENVELOPE_ASSIGN << 1;
  // NOLINTEND(*-magic-numbers)

  if (ck_process_kings_letter(&letter) != CK_OK) {
    printf("process_kp2: assign: failed to process page.\n");
    return TEST_FAIL;
  }

  uint32_t got_envelope_no = data.folders[2].envelopes[0].envelope_no;
  if (got_envelope_no != envelope2) {
    printf("process_kp2: assign: wrong envelope no, expected: %u, got: %u.\n",
           envelope2, got_envelope_no);
    return TEST_FAIL;
  }

  if (data.folders[2].envelope_count != 1) {
    printf(
        "process_kp2: assign: wrong envelope count for folder 2, "
        "expected: 1, got: %u.\n",
        data.folders[2].envelope_count);
    return TEST_FAIL;
  }

  // Assign envelope no 3 to folder no 2.
  uint32_t envelope3 = 3;
  memcpy(&letter.page.lines[2], &envelope3, sizeof(envelope3));
  if (ck_process_kings_letter(&letter) != CK_OK) {
    printf("process_kp2: assign: failed to process page.\n");
    return TEST_FAIL;
  }

  got_envelope_no = data.folders[2].envelopes[1].envelope_no;
  if (got_envelope_no != envelope3) {
    printf("process_kp2: assign: wrong envelope no, expected: %u, got: %u.\n",
           envelope3, got_envelope_no);
    return TEST_FAIL;
  }

  if (data.folders[2].envelope_count != 2) {
    printf(
        "process_kp2: assign: wrong envelope count for folder 2, "
        "expected: 2, got: %u.\n",
        data.folders[2].envelope_count);
    return TEST_FAIL;
  }

  // Transfer envelope no 3 to folder no 3.
  uint32_t folder3 = 3;
  memcpy(&letter.page.lines[2], &envelope3, sizeof(envelope3));
  // NOLINTBEGIN(*-magic-numbers)
  letter.page.lines[6] = folder3;
  letter.page.lines[7] = CK_ENVELOPE_TRANSFER << 1 | 1;  // Transfer and enable
  // NOLINTEND(*-magic-numbers)
  if (ck_process_kings_letter(&letter) != CK_OK) {
    printf("process_kp2: transfer: failed to process page.\n");
    return TEST_FAIL;
  }
  got_envelope_no = data.folders[3].envelopes[0].envelope_no;
  if (got_envelope_no != envelope3) {
    printf("process_kp2: transfer: wrong envelope no, expected: %u, got: %u.\n",
           envelope3, got_envelope_no);
    return TEST_FAIL;
  }
  if (data.folders[2].envelope_count != 1) {
    printf(
        "process_kp2: transfer: wrong envelope count for folder 2, "
        "expected: 1, got: %u.\n",
        data.folders[2].envelope_count);
    return TEST_FAIL;
  }

  if (data.folders[3].envelope_count != 1) {
    printf(
        "process_kp2: transfer: wrong envelope count for folder 3, "
        "expected: 1, got: %u.\n",
        data.folders[3].envelope_count);
    return TEST_FAIL;
  }
  if (!data.folders[3].envelopes[0].enable) {
    printf("process_kp2: transfer: envelope 3 not enabled.\n");
    return TEST_FAIL;
  }

  // Expel envelope no 2.
  memcpy(&letter.page.lines[2], &envelope2, sizeof(envelope2));
  // NOLINTBEGIN(*-magic-numbers)
  letter.page.lines[6] = 0;  // Ignored
  letter.page.lines[7] = CK_ENVELOPE_EXPEL << 1;
  // NOLINTEND(*-magic-numbers)
  if (ck_process_kings_letter(&letter) != CK_OK) {
    printf("process_kp2: expel: failed to process page.\n");
    return TEST_FAIL;
  }
  if (data.folders[2].envelope_count > 0) {
    printf(
        "process_kp2: expel: wrong envelope count for folder 2, "
        "expected: 0, got: %u.\n",
        data.folders[2].envelope_count);
    return TEST_FAIL;
  }

  return TEST_PASS;
}

// Moves the document in folder 2 to folder 3.
static test_err_t test_process_kp16(void) {
  // Places the tx doc in folder 2 and rx doc in folder 3.
  test_err_t err = setup_test();
  if (err != TEST_PASS) {
    return err;
  }

  // NOLINTBEGIN(*-magic-numbers)
  // Insert document T0.1 into folder 3.
  ck_letter_t letter;
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

  if (ck_process_kings_letter(&letter) != CK_OK) {
    printf("process_kp16: insert: failed to process page.\n");
    return TEST_FAIL;
  }
  if (data.folders[3].dlc != dlc) {
    printf("process_kp16: insert: wrong DLC, expected: %u, got: %u.\n", dlc,
           data.folders[3].dlc);
    return TEST_FAIL;
  }
  if (data.folders[3].direction != CK_DIRECTION_TRANSMIT) {
    printf("process_kp16: insert: wrong direction, expected: %u, got: %u.\n",
           CK_DIRECTION_TRANSMIT, data.folders[3].direction);
    return TEST_FAIL;
  }
  if (data.folders[3].enable) {
    printf("process_kp16: insert: folder should have been disabled.\n");
    return TEST_FAIL;
  }
  if (data.folders[3].doc_list_no != list_no) {
    printf("process_kp16: insert: wrong list no, expected: %u, got: %u.\n",
           list_no, data.folders[3].doc_list_no);
    return TEST_FAIL;
  }
  if (data.folders[3].doc_no != doc_no) {
    printf("process_kp16: insert: wrong doc no, expected: %u, got: %u.\n",
           doc_no, data.folders[3].doc_no);
    return TEST_FAIL;
  }

  // Remove document from folder 2
  *folder_no = 2;
  if (ck_process_kings_letter(&letter) != CK_OK) {
    printf("process_kp16: remove: failed to process page.\n");
    return TEST_FAIL;
  }
  if (data.folders[2].enable) {
    printf("process_kp16: remove: folder should have been disabled.\n");
    return TEST_FAIL;
  }

  // Test illegal parameters
  *folder_no = 0;
  if (ck_process_kings_letter(&letter) == CK_OK) {
    printf("process_kp16: illegal folder no 0 returned OK.\n");
    return TEST_FAIL;
  }

  *folder_no = FOLDER_COUNT + 1;
  if (ck_process_kings_letter(&letter) == CK_OK) {
    printf("process_kp16: non-existent folder returned OK.\n");
    return TEST_FAIL;
  }

  return TEST_PASS;
}

// Test creating a line, creating a page, and creating a document.
static test_err_t test_process_kp17(void) {
  test_err_t err = setup_test();
  if (err != TEST_PASS) {
    return err;
  }

  ck_letter_t letter;
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

  if (ck_process_kings_letter(&letter) == CK_OK) {
    printf("process_kp17: illegal parameter returned OK.\n");
    return TEST_FAIL;
  }

  letter.page.lines[2] = CK_LIST_LINE << 3;
  if (ck_process_kings_letter(&letter) == CK_OK) {
    printf("process_kp17: illegal parameter returned OK.\n");
    return TEST_FAIL;
  }

  *source_list_number = 0;
  if (ck_process_kings_letter(&letter) == CK_OK) {
    printf("process_kp17: illegal parameter returned OK.\n");
    return TEST_FAIL;
  }

  *target_record_number = 0;
  if (ck_process_kings_letter(&letter) == CK_OK) {
    printf("process_kp17: illegal parameter returned OK.\n");
    return TEST_FAIL;
  }

  // Create line R0.0 using Bit R0.0. Should have value 1 (0b01).
  *target_position = 0;
  if (ck_process_kings_letter(&letter) != CK_OK) {
    printf("process_kp17: line: failed to process page.\n");
    return TEST_FAIL;
  }
  // Create line R0.1 using Bit R0.0. Should have value 2 (0b10).
  *target_record_number = 1;
  *target_position = 1;
  if (ck_process_kings_letter(&letter) != CK_OK) {
    printf("process_kp17: line: failed to process page.\n");
    return TEST_FAIL;
  }

  // Verify the data
  uint8_t *line_r00 = (uint8_t *)data.rx_line_list->records[0];
  uint8_t *line_r01 = (uint8_t *)data.rx_line_list->records[1];
  if (*line_r00 != 1 || *line_r01 != 2) {
    printf("process_kp17: wrong line data, expected: 1, 2, got: %u, %u.\n",
           *line_r00, *line_r01);
    return TEST_FAIL;
  }

  // Create page R0.1 with lines R0.0 and R0.1
  letter.page.lines[2] = CK_LIST_PAGE << 3;
  *target_record_number = 1;
  *source_record_number = 0;
  *target_position = 0;
  if (ck_process_kings_letter(&letter) != CK_OK) {
    printf("process_kp17: page: failed to process page.\n");
    return TEST_FAIL;
  }

  *source_record_number = 1;
  *target_position = 1;
  if (ck_process_kings_letter(&letter) != CK_OK) {
    printf("process_kp17: page: failed to process page.\n");
    return TEST_FAIL;
  }

  // Verify the data
  ck_page_t *page_r01 = (ck_page_t *)data.rx_page_list->records[1];
  if (page_r01->line_count != 2) {
    printf("process_kp17: wrong line count, expected: 2, got: %u.\n",
           page_r01->line_count);
    return TEST_FAIL;
  }
  if (page_r01->lines[0] != 1 || page_r01->lines[1] != 2) {
    printf("process_kp17: wrong page data, expected: 1, 2, got: %u, %u.\n",
           page_r01->lines[0], page_r01->lines[1]);
    return TEST_FAIL;
  }

  // Create document R0.1 which contains two copies of page R0.1.
  letter.page.lines[2] = CK_LIST_DOCUMENT << 3;
  *target_record_number = 1;
  *source_record_number = 1;
  *target_position = 0;
  if (ck_process_kings_letter(&letter) != CK_OK) {
    printf("process_kp17: document: failed to process page.\n");
    return TEST_FAIL;
  }
  *target_position = 1;
  if (ck_process_kings_letter(&letter) != CK_OK) {
    printf("process_kp17: document: failed to process page.\n");
    return TEST_FAIL;
  }

  // Verify the data
  ck_document_t *doc_r01 = (ck_document_t *)data.rx_doc_list->records[1];
  if (doc_r01->page_count != 2) {
    printf("process_kp17: wrong page count, expected: 2, got: %u.\n",
           doc_r01->page_count);
    return TEST_FAIL;
  }
  if (doc_r01->pages[0]->lines[0] != 1 || doc_r01->pages[0]->lines[1] != 2 ||
      doc_r01->pages[1]->lines[0] != 1 || doc_r01->pages[1]->lines[1] != 2) {
    printf(
        "process_kp17: wrong page data, expected: 1, 2, 1, 2, "
        "got: %u, %u, %u, %u.\n ",
        doc_r01->pages[0]->lines[0], doc_r01->pages[0]->lines[1],
        doc_r01->pages[1]->lines[0], doc_r01->pages[1]->lines[1]);
    return TEST_FAIL;
  }

  return TEST_PASS;
}

static ck_err_t set_action_mode(ck_action_mode_t mode) {
  (void)mode;
  return CK_OK;
}

static ck_err_t set_comm_mode(ck_comm_mode_t mode) {
  (void)mode;
  return CK_OK;
}
static ck_err_t set_city_mode(ck_city_mode_t mode) {
  (void)mode;
  return CK_OK;
}

static test_err_t check_kings_doc_folder(ck_folder_t *folder) {
  if (folder->envelope_count != 1) {
    printf(
        "mayor_init: failed to setup folder for king's doc: "
        "wrong envelope count, expected: 1, got: %u.\n",
        folder->envelope_count);
    return TEST_FAIL;
  }
  if (folder->envelopes[0].envelope_no != 0) {
    printf(
        "mayor_init: failed to setup folder for king's doc: "
        "wrong envelope number, expected: 0, got: %u.\n",
        folder->envelopes[0].envelope_no);
    return TEST_FAIL;
  }
  if (!folder->envelopes[0].enable) {
    printf(
        "mayor_init: failed to setup folder for king's doc: "
        "envelope not enabled.\n");
    return TEST_FAIL;
  }
  if (folder->dlc != CK_CAN_MAX_DLC) {
    printf(
        "mayor_init: failed to setup folder for king's doc: "
        "wrong DLC, expected: %u, got: %u.\n",
        CK_CAN_MAX_DLC, folder->dlc);
    return TEST_FAIL;
  }
  if (!folder->enable) {
    printf(
        "mayor_init: failed to setup folder for king's doc: "
        "folder not enabled.\n");
    return TEST_FAIL;
  }
  if (folder->direction != CK_DIRECTION_RECEIVE) {
    printf(
        "mayor_init: failed to setup folder for king's doc: "
        "wrong direction, expected: %u, got: %u.\n",
        CK_DIRECTION_RECEIVE, folder->direction);
    return TEST_FAIL;
  }
  return TEST_PASS;
}

static test_err_t check_mayors_doc_folder(ck_folder_t *folder) {
  if (folder->envelope_count != 1) {
    printf(
        "mayor_init: failed to setup folder for mayors's doc: "
        "wrong envelope count, expected: 1, got: %u.\n",
        folder->envelope_count);
    return TEST_FAIL;
  }
  if (folder->envelopes[0].envelope_no != test_base_no + test_city_address) {
    printf(
        "mayor_init: failed to setup folder for mayors's doc: "
        "wrong envelope number, expected: %u, got: %u.\n",
        test_base_no + test_city_address, folder->envelopes[0].envelope_no);
    return TEST_FAIL;
  }
  if (!folder->envelopes[0].enable) {
    printf(
        "mayor_init: failed to setup folder for mayors's doc: "
        "envelope not enabled.\n");
    return TEST_FAIL;
  }
  if (folder->dlc != CK_CAN_MAX_DLC) {
    printf(
        "mayor_init: failed to setup folder for mayors's doc: "
        "wrong DLC, expected: %u, got: %u.\n",
        CK_CAN_MAX_DLC, folder->dlc);
    return TEST_FAIL;
  }
  if (!folder->enable) {
    printf(
        "mayor_init: failed to setup folder for mayors's doc: "
        "folder not enabled.\n");
    return TEST_FAIL;
  }
  if (folder->direction != CK_DIRECTION_TRANSMIT) {
    printf(
        "mayor_init: failed to setup folder for mayors's doc: "
        "wrong direction, expected: %u, got: %u.\n",
        CK_DIRECTION_TRANSMIT, folder->direction);
    return TEST_FAIL;
  }
  return TEST_PASS;
}

static void init_data(void) {
  memset(&data, 0, sizeof(data));
  data.rx_bits[0] = 1;
  init_pages();
  init_docs();
  init_lists();
  init_folders();
}

static void init_pages(void) {
  data.rx_pages[0].line_count = CK_MAX_LINES_PER_PAGE;
  const uint8_t rx_line[8] = {7, 6, 5, 4, 3, 2, 1, 0};
  memcpy(data.rx_pages[0].lines, rx_line, sizeof(rx_line));
}

static void init_docs(void) {
  data.rx_docs[0].direction = CK_DIRECTION_RECEIVE;
  data.rx_docs[0].page_count = 1;
  data.rx_docs[0].pages[0] = data.rx_pages;

  data.tx_docs[0].direction = CK_DIRECTION_TRANSMIT;
  data.tx_docs[0].page_count = 0;
}

static void init_lists(void) {
  // Transmit doc list number 0. Required by the mayor lib, to create the
  // mayor's document. We also add a test document for testing
  // ck_send_document().
  data.tx_doc_list = &data.lists[0];
  data.tx_doc_list->type = CK_LIST_DOCUMENT;
  data.tx_doc_list->direction = CK_DIRECTION_TRANSMIT;
  data.tx_doc_list->list_no = 0;
  data.tx_doc_list->record_count = 1 + TX_DOCUMENT_COUNT;
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

static void init_folders(void) {
  // Set up folders 2 and 3. 0 and 1 are reserved for the king's doc and mayor's
  // doc, which are set up by the mayor lib.
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

static test_err_t setup_test(void) {
  init_data();
  ck_mayor_t mayor = {
      .ean_no = test_ean_no,
      .serial_no = test_ean_no,
      .city_address = test_city_address,
      .base_no = test_base_no,
      .has_extended_id = false,
      .set_action_mode = set_action_mode,
      .set_comm_mode = set_comm_mode,
      .set_city_mode = set_city_mode,
      .folder_count = FOLDER_COUNT,
      .folders = data.folders,
      .list_count = LIST_COUNT,
      .lists = data.lists,
  };

  if (ck_mayor_init(&mayor) != CK_OK) {
    return TEST_SETUP_FAIL;
  }

  return TEST_PASS;
}
