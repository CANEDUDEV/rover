#include "ck-data.h"

static ck_data_t ck_data;

void page_init(void);
void doc_init(void);
void list_init(void);
void folder_init(void);

void ck_data_init(void) {
  page_init();
  doc_init();
  list_init();
  folder_init();
}

ck_data_t* get_ck_data(void) { return &ck_data; }

void page_init(void) {
  ck_data.steering_page = &ck_data.pages[0];
  ck_data.throttle_page = &ck_data.pages[1];
  ck_data.steering_subtrim_page = &ck_data.pages[2];
  ck_data.throttle_subtrim_page = &ck_data.pages[3];

  ck_data.steering_page->lines[0] = 1;
  ck_data.throttle_page->lines[0] = 0;

  ck_data.steering_page->line_count = 5;  // NOLINT
  ck_data.throttle_page->line_count = 3;
  ck_data.steering_subtrim_page->line_count = 2;
  ck_data.throttle_subtrim_page->line_count = 2;
}

void doc_init(void) {
  for (uint8_t i = 0; i < CK_DATA_TX_DOC_COUNT; i++) {
    ck_data.docs[i].direction = CK_DIRECTION_TRANSMIT;
    ck_data.docs[i].page_count = 1;
    ck_data.docs[i].pages[0] = &ck_data.pages[i];
  }
}

void list_init(void) {
  ck_data.tx_list = &ck_data.lists[0];
  ck_data.rx_list = &ck_data.lists[1];

  // Set up the doc lists
  ck_data.rx_list->type = CK_LIST_DOCUMENT;
  ck_data.rx_list->direction = CK_DIRECTION_RECEIVE;
  ck_data.rx_list->list_no = 0;
  ck_data.rx_list->record_count = 1;  // Only 1 slot, for the king's doc.

  ck_data.tx_list->type = CK_LIST_DOCUMENT;
  ck_data.tx_list->direction = CK_DIRECTION_TRANSMIT;
  ck_data.tx_list->list_no = 0;

  // CK needs 1 slot for the mayor's doc.
  ck_data.tx_list->record_count = CK_DATA_TX_DOC_COUNT + 1;

  for (uint8_t i = 0; i < CK_DATA_TX_DOC_COUNT; i++) {
    ck_data.tx_list->records[i + 1] = &ck_data.docs[i];
  }
}

void folder_init(void) {
  ck_data.steering_folder = &ck_data.folders[2];
  ck_data.throttle_folder = &ck_data.folders[3];
  ck_data.steering_subtrim_folder = &ck_data.folders[4];
  ck_data.throttle_subtrim_folder = &ck_data.folders[5];  // NOLINT

  // Set up the transmit folders
  for (int i = 2; i < 2 + CK_DATA_TX_FOLDER_COUNT; i++) {
    ck_data.folders[i].folder_no = i;
    ck_data.folders[i].direction = CK_DIRECTION_TRANSMIT;
    ck_data.folders[i].doc_list_no = 0;
    ck_data.folders[i].doc_no = i - 1;  // 0 reserved by mayor's doc
    ck_data.folders[i].enable = true;
  }

  ck_data.steering_folder->dlc = 5;  // NOLINT
  ck_data.throttle_folder->dlc = 3;
  ck_data.steering_subtrim_folder->dlc = 2;
  ck_data.throttle_subtrim_folder->dlc = 2;
}
