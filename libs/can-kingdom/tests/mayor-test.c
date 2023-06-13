#include "mayor.h"

#include <stdio.h>
#include <string.h>

#include "king.h"
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
static test_err_t test_send_document(void);
static test_err_t test_send_mayors_page(void);
static test_err_t test_is_kings_envelope(void);
static test_err_t test_get_envelopes_folder(void);
static test_err_t test_set_comm_mode(void);
static test_err_t test_set_base_number(void);
static test_err_t test_is_default_letter(void);
static test_err_t test_default_letter_received(void);
static test_err_t test_process_kp0(void);
static test_err_t test_process_kp1(void);
static test_err_t test_process_kp2(void);
static test_err_t test_process_kp8(void);
static test_err_t test_process_kp16(void);
static test_err_t test_process_kp17(void);

// Helpers
static ck_err_t set_action_mode(ck_action_mode_t mode);
static ck_err_t set_city_mode(ck_city_mode_t mode);
static void start_200ms_timer(void);

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

  test_err_t ret = test_mayor_init();
  if (ret != TEST_PASS) {
    return ret;
  }

  ret = test_process_kings_letter();
  if (ret != TEST_PASS) {
    return ret;
  }

  ret = test_add_mayors_page();
  if (ret != TEST_PASS) {
    return ret;
  }

  ret = test_send_document();
  if (ret != TEST_PASS) {
    return ret;
  }

  ret = test_send_mayors_page();
  if (ret != TEST_PASS) {
    return ret;
  }

  ret = test_is_kings_envelope();
  if (ret != TEST_PASS) {
    return ret;
  }

  ret = test_get_envelopes_folder();
  if (ret != TEST_PASS) {
    return ret;
  }

  ret = test_set_comm_mode();
  if (ret != TEST_PASS) {
    return ret;
  }

  ret = test_set_base_number();
  if (ret != TEST_PASS) {
    return ret;
  }

  ret = test_is_default_letter();
  if (ret != TEST_PASS) {
    return ret;
  }

  ret = test_default_letter_received();
  if (ret != TEST_PASS) {
    return ret;
  }

  return TEST_PASS;
}

static test_err_t test_mayor_init(void) {
  init_data();

  // Set up with some illegal parameters first.
  ck_mayor_t mayor = {
      .ean_no = illegal_ean_no,  // Illegal EAN
      .serial_no = test_serial_no,
      .city_address = 0,  // Illegal city address
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
  err = test_process_kp0();
  if (err != TEST_PASS) {
    return err;
  }
  err = test_process_kp1();
  if (err != TEST_PASS) {
    return err;
  }
  err = test_process_kp2();
  if (err != TEST_PASS) {
    return err;
  }
  err = test_process_kp8();
  if (err != TEST_PASS) {
    return err;
  }
  err = test_process_kp16();
  if (err != TEST_PASS) {
    return err;
  }
  err = test_process_kp17();
  if (err != TEST_PASS) {
    return err;
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

static test_err_t test_send_document(void) {
  test_err_t err = setup_test();
  if (err != TEST_PASS) {
    return err;
  }

  ck_set_comm_mode(CK_COMM_MODE_COMMUNICATE);

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

static test_err_t test_send_mayors_page(void) {
  test_err_t err = setup_test();
  if (err != TEST_PASS) {
    return err;
  }

  ck_set_comm_mode(CK_COMM_MODE_COMMUNICATE);

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

static test_err_t test_is_kings_envelope(void) {
  ck_envelope_t good_envelope = {.envelope_no = 0};
  ck_envelope_t bad_envelope = {.envelope_no = 1};

  if (ck_is_kings_envelope(&good_envelope) != CK_OK) {
    printf("is_kings_envelope: king's envelope returned error.\n");
    return TEST_FAIL;
  }

  if (ck_is_kings_envelope(&bad_envelope) == CK_OK) {
    printf("is_kings_envelope: bad envelope returned OK.\n");
    return TEST_FAIL;
  }

  return TEST_PASS;
}

// First, check if envelope 200 is assigned to folder 2. Then, assign it, and
// test again.
static test_err_t test_get_envelopes_folder(void) {
  test_err_t err = setup_test();
  if (err != TEST_PASS) {
    return err;
  }
  ck_envelope_t envelope = {.envelope_no = 200};  // NOLINT

  ck_folder_t *folder = NULL;

  if (ck_get_envelopes_folder(&envelope, &folder) != CK_ERR_FALSE) {
    printf(
        "get_envelopes_folder: checking for non-existent envelope did not "
        "return false.\n");
    return TEST_FAIL;
  }

  ck_kp2_args_t args = {
      .folder_no = 2,
      .envelope = envelope,
      .envelope_action = CK_ENVELOPE_ASSIGN,
  };
  ck_letter_t letter;

  ck_create_kings_page_2(&args, &letter.page);

  if (ck_process_kings_letter(&letter) != CK_OK) {
    printf("get_envelopes_folder: failed to assign envelope.\n");
    return TEST_FAIL;
  }

  if (ck_get_envelopes_folder(&envelope, &folder) != CK_OK) {
    printf(
        "get_envelopes_folder: checking for existent envelope did not "
        "return OK.\n");
    return TEST_FAIL;
  }

  if (folder != &data.folders[2] ||
      folder->folder_no != data.folders[2].folder_no) {
    printf(
        "get_envelopes_folder: returned wrong folder, expected: %u, got: %u\n",
        data.folders[2].folder_no, folder->folder_no);
    return TEST_FAIL;
  }

  return TEST_PASS;
}

static test_err_t test_set_comm_mode(void) {
  test_err_t ret = setup_test();
  if (ret != TEST_PASS) {
    return ret;
  }
  if (ck_get_comm_mode() != CK_COMM_MODE_SILENT) {
    printf("set_comm_mode: comm mode not init to CK_COMM_MODE_SILENT.\n");
    return TEST_FAIL;
  }
  if (ck_set_comm_mode(CK_COMM_MODE_COMMUNICATE) != CK_OK) {
    printf("set_comm_mode: failed to set comm mode.\n");
    return TEST_FAIL;
  }
  ck_comm_mode_t mode = ck_get_comm_mode();
  if (mode != CK_COMM_MODE_COMMUNICATE) {
    printf(
        "set_comm_mode: get comm mode returned wrong comm mode, "
        "expected: %u, got: %u.\n",
        CK_COMM_MODE_COMMUNICATE, mode);
    return TEST_FAIL;
  }
  return TEST_PASS;
}

static test_err_t test_set_base_number(void) {
  test_err_t ret = setup_test();
  if (ret != TEST_PASS) {
    return ret;
  }
  if (ck_set_base_number(test_base_no, false) != CK_OK) {
    printf("set_base_number: failed to set base number.\n");
    return TEST_FAIL;
  }
  uint32_t got_base_no = ck_get_base_number();
  if (got_base_no != test_base_no) {
    printf(
        "set_base_number: get base number returned wrong base number, "
        "expected: %u, got: %u.\n",
        test_base_no, got_base_no);
    return TEST_FAIL;
  }
  return TEST_PASS;
}

static test_err_t test_is_default_letter(void) {
  ck_letter_t dletter = ck_default_letter();
  if (ck_is_default_letter(&dletter) != CK_OK) {
    return TEST_FAIL;
  }
  ck_letter_t illegal_letter = {
      .envelope.envelope_no = 0,
  };
  if (ck_is_default_letter(&illegal_letter) == CK_OK) {
    return TEST_FAIL;
  }
  return TEST_PASS;
}

static test_err_t test_default_letter_received(void) {
  test_err_t ret = setup_test();
  if (ret != TEST_PASS) {
    return ret;
  }

  if (ck_default_letter_received() != CK_OK) {
    return TEST_FAIL;
  }
  ck_comm_mode_t mode = ck_get_comm_mode();
  if (mode != CK_COMM_MODE_LISTEN_ONLY) {
    printf(
        "default_letter_received: comm mode not correct after default letter "
        "reception, expected: %u, got: %u.\n",
        CK_COMM_MODE_COMMUNICATE, mode);
    return TEST_FAIL;
  }

  ret = setup_test();
  if (ret != TEST_PASS) {
    return ret;
  }
  // Timeout before default letter is received
  if (ck_default_letter_timeout() != CK_OK) {
    return TEST_FAIL;
  }
  if (ck_default_letter_received() != CK_OK) {
    return TEST_FAIL;
  }
  if (ck_get_comm_mode() != CK_COMM_MODE_SILENT) {
    printf(
        "default_letter_received: comm mode changed even though default "
        "letter was received after timeout.\n");
    return TEST_FAIL;
  }

  return TEST_PASS;
}

static test_err_t test_process_kp0(void) {
  ck_kp0_args_t args = {
      .address = test_city_address,
      .action_mode = CK_ACTION_MODE_FREEZE,
      .comm_mode = CK_COMM_MODE_SILENT,
      .comm_flags = CK_COMM_RESET,
      .city_mode = CK_CITY_MODE_KEEP_CURRENT,
  };

  ck_letter_t letter;
  ck_err_t ret = ck_create_kings_page_0(&args, &letter.page);
  if (ret != CK_OK) {
    printf("process_kp0: failed to create king's page.\n");
    return TEST_FAIL;
  }

  if (ck_process_kings_letter(&letter) != CK_OK) {
    printf("process_kp0: failed to process page.\n");
    return TEST_FAIL;
  }

  return TEST_PASS;
}

static test_err_t test_process_kp1(void) {
  ck_kp1_args_t args = {
      .address = test_city_address,
      .base_no = test_base_no,
      .mayor_response_no = CK_NO_RESPONSE_REQUESTED,
  };
  ck_letter_t letter;
  ck_create_kings_page_1(&args, &letter.page);

  if (ck_process_kings_letter(&letter) != CK_OK) {
    printf("process_kp1: failed to process page.\n");
    return TEST_FAIL;
  }

  // Invalid base no. (base no + city address will be too big)
  uint32_t invalid_base_no = illegal_can_std_id - 1;
  args.base_no = invalid_base_no;
  ck_create_kings_page_1(&args, &letter.page);

  if (ck_process_kings_letter(&letter) == CK_OK) {
    printf("process_kp1: invalid base no page returned OK.\n");
    return TEST_FAIL;
  }

  // Invalid base no (extended ID)
  invalid_base_no = illegal_can_ext_id - 1;
  args.base_no = invalid_base_no;
  args.has_extended_id = true;
  ck_create_kings_page_1(&args, &letter.page);

  if (ck_process_kings_letter(&letter) == CK_OK) {
    printf("process_kp1: invalid ext base no page returned OK.\n");
    return TEST_FAIL;
  }

  // Invalid mayor response page
  args.base_no = test_base_no;
  args.mayor_response_no = 3;
  ck_create_kings_page_1(&args, &letter.page);

  ck_set_comm_mode(CK_COMM_MODE_COMMUNICATE);

  if (ck_process_kings_letter(&letter) == CK_OK) {
    printf("process_kp1: invalid mayor's response page returned OK.\n");
    return TEST_FAIL;
  }

  return TEST_PASS;
}

// First assign envelopes 200 and 300 to folder 2, then transfer envelope 300 to
// folder 3, then expel envelope 200.
static test_err_t test_process_kp2(void) {
  test_err_t err = setup_test();
  if (err != TEST_PASS) {
    return err;
  }

  // Assign envelope no 200 to folder no 2.
  ck_kp2_args_t args = {
      .address = 0,
      .folder_no = 2,
      .envelope.envelope_no = 200,  // NOLINT
      .envelope_action = CK_ENVELOPE_ASSIGN,
  };
  ck_letter_t letter;

  ck_create_kings_page_2(&args, &letter.page);

  if (ck_process_kings_letter(&letter) != CK_OK) {
    printf("process_kp2: assign: failed to process page.\n");
    return TEST_FAIL;
  }

  uint32_t got_envelope_no = data.folders[2].envelopes[0].envelope_no;
  if (got_envelope_no != args.envelope.envelope_no) {
    printf("process_kp2: assign: wrong envelope no, expected: %u, got: %u.\n",
           args.envelope.envelope_no, got_envelope_no);
    return TEST_FAIL;
  }

  if (data.folders[2].envelope_count != 1) {
    printf(
        "process_kp2: assign: wrong envelope count for folder 2, "
        "expected: 1, got: %u.\n",
        data.folders[2].envelope_count);
    return TEST_FAIL;
  }

  // Assign envelope no 300 to folder no 2.
  args.folder_no = 2;
  args.envelope.envelope_no = 300;  // NOLINT
  ck_create_kings_page_2(&args, &letter.page);

  if (ck_process_kings_letter(&letter) != CK_OK) {
    printf("process_kp2: assign: failed to process page.\n");
    return TEST_FAIL;
  }

  got_envelope_no = data.folders[2].envelopes[1].envelope_no;
  if (got_envelope_no != args.envelope.envelope_no) {
    printf("process_kp2: assign: wrong envelope no, expected: %u, got: %u.\n",
           args.envelope.envelope_no, got_envelope_no);
    return TEST_FAIL;
  }

  if (data.folders[2].envelope_count != 2) {
    printf(
        "process_kp2: assign: wrong envelope count for folder 2, "
        "expected: 2, got: %u.\n",
        data.folders[2].envelope_count);
    return TEST_FAIL;
  }

  // Transfer envelope no 300 to folder no 3, and enable it.
  args.folder_no = 3;
  args.envelope_action = CK_ENVELOPE_TRANSFER;
  args.envelope.envelope_no = 300;  // NOLINT
  args.envelope.enable = true;
  ck_create_kings_page_2(&args, &letter.page);

  if (ck_process_kings_letter(&letter) != CK_OK) {
    printf("process_kp2: transfer: failed to process page.\n");
    return TEST_FAIL;
  }
  got_envelope_no = data.folders[3].envelopes[0].envelope_no;
  if (got_envelope_no != args.envelope.envelope_no) {
    printf("process_kp2: transfer: wrong envelope no, expected: %u, got: %u.\n",
           args.envelope.envelope_no, got_envelope_no);
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
    printf("process_kp2: transfer: envelope 300 not enabled.\n");
    return TEST_FAIL;
  }

  // Expel envelope no 200.
  args.envelope.envelope_no = 200;  // NOLINT
  args.envelope_action = CK_ENVELOPE_EXPEL;
  args.envelope.enable = false;
  ck_create_kings_page_2(&args, &letter.page);

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

static test_err_t test_process_kp8(void) {
  // This sets up the mayor with the default bit timing.
  test_err_t err = setup_test();
  if (err != TEST_PASS) {
    return err;
  }

  ck_can_bit_timing_t next_bit_timing = {
      .prescaler = 10,    // NOLINT
      .time_quanta = 10,  // NOLINT
      .phase_seg2 = 4,
      .sjw = 2,
  };

  ck_letter_t letter;
  if (ck_create_kings_page_8(test_city_address, &next_bit_timing,
                             &letter.page) != CK_OK) {
    printf("process_kp8: failed to create KP8.\n");
    return TEST_FAIL;
  }

  if (ck_process_kings_letter(&letter) != CK_OK) {
    printf("process_kp8: failed to process KP8.\n");
    return TEST_FAIL;
  }

  // Send kp0 reset comm message
  ck_kp0_args_t args = {
      .address = test_city_address,
      .action_mode = CK_ACTION_MODE_KEEP_CURRENT,
      .comm_mode = CK_COMM_MODE_KEEP_CURRENT,
      .comm_flags = CK_COMM_RESET,
      .city_mode = CK_CITY_MODE_KEEP_CURRENT,
  };

  if (ck_create_kings_page_0(&args, &letter.page) != CK_OK) {
    printf("process_kp8: failed to create KP0.\n");
    return TEST_FAIL;
  }

  if (ck_process_kings_letter(&letter) != CK_OK) {
    printf("process_kp8: failed to process KP0.\n");
    return TEST_FAIL;
  }

  // Emulate reception of a letter, so the bit timing is saved persistently.
  if (ck_correct_letter_received() != CK_OK) {
    return TEST_SETUP_FAIL;
  }

  ck_can_bit_timing_t current_bit_timing;
  if (ck_load_bit_timing(&current_bit_timing) != CK_OK) {
    printf("process_kp8: failed to load current bit timing.\n");
    return TEST_FAIL;
  }

  if (memcmp(&current_bit_timing, &next_bit_timing,
             sizeof(ck_can_bit_timing_t)) != 0) {
    printf("process_kp8: bit timing was not set correctly.\n");
    printf("expected: prescaler: %u, tq: %u, ps2: %u, sjw: %u.\n",
           next_bit_timing.prescaler, next_bit_timing.time_quanta,
           next_bit_timing.phase_seg2, next_bit_timing.sjw);
    printf("got: prescaler: %u, tq: %u, ps2: %u, sjw: %u.\n",
           current_bit_timing.prescaler, current_bit_timing.time_quanta,
           current_bit_timing.phase_seg2, current_bit_timing.sjw);
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

static ck_err_t set_city_mode(ck_city_mode_t mode) {
  (void)mode;
  return CK_OK;
}

static void start_200ms_timer(void) {}

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
  if (folder->envelopes[0].enable) {
    printf(
        "mayor_init: failed to setup folder for mayor's doc: "
        "envelope is enabled before setting base number.\n");
    return TEST_FAIL;
  }

  if (ck_set_base_number(test_base_no, false) != CK_OK) {
    printf(
        "mayor_init: failed to setup folder for mayor's doc: "
        "couldn't set base number.\n");
    return TEST_FAIL;
  }

  if (folder->envelope_count != 1) {
    printf(
        "mayor_init: failed to setup folder for mayor's doc: "
        "wrong envelope count, expected: 1, got: %u.\n",
        folder->envelope_count);
    return TEST_FAIL;
  }

  if (folder->envelopes[0].envelope_no != test_base_no + test_city_address) {
    printf(
        "mayor_init: failed to setup folder for mayor's doc: "
        "wrong envelope number, expected: %u, got: %u.\n",
        test_base_no + test_city_address, folder->envelopes[0].envelope_no);
    return TEST_FAIL;
  }
  if (!folder->envelopes[0].enable) {
    printf(
        "mayor_init: failed to setup folder for mayor's doc: "
        "envelope not enabled.\n");
    return TEST_FAIL;
  }
  if (folder->dlc != CK_CAN_MAX_DLC) {
    printf(
        "mayor_init: failed to setup folder for mayor's doc: "
        "wrong DLC, expected: %u, got: %u.\n",
        CK_CAN_MAX_DLC, folder->dlc);
    return TEST_FAIL;
  }
  if (!folder->enable) {
    printf(
        "mayor_init: failed to setup folder for mayor's doc: "
        "folder not enabled.\n");
    return TEST_FAIL;
  }
  if (folder->direction != CK_DIRECTION_TRANSMIT) {
    printf(
        "mayor_init: failed to setup folder for mayor's doc: "
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

static test_err_t setup_test(void) {
  init_data();
  ck_mayor_t mayor = {
      .ean_no = test_ean_no,
      .serial_no = test_ean_no,
      .city_address = test_city_address,
      .set_action_mode = set_action_mode,
      .set_city_mode = set_city_mode,
      .start_200ms_timer = start_200ms_timer,
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
