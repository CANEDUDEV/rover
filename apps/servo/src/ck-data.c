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
  ck_data.sensor_power_page = &ck_data.pages[0];
  ck_data.servo_current_page = &ck_data.pages[1];
  ck_data.battery_voltage_page = &ck_data.pages[2];
  ck_data.servo_voltage_page = &ck_data.pages[3];
  ck_data.h_bridge_current_page = &ck_data.pages[4];

  // Set up the pages
  for (uint8_t i = 0; i < CK_DATA_TX_PAGE_COUNT; i++) {
    ck_data.pages[i].line_count = 2;
  }
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
  // NOLINTBEGIN(*-magic-numbers)
  ck_data.sensor_power_folder = &ck_data.folders[2];
  ck_data.servo_current_folder = &ck_data.folders[3];
  ck_data.battery_voltage_folder = &ck_data.folders[4];
  ck_data.servo_voltage_folder = &ck_data.folders[5];
  ck_data.h_bridge_current_folder = &ck_data.folders[6];

  // Set up the transmit folders
  for (int i = 2; i < 2 + 5; i++) {  // 5 transmit folders
    ck_data.folders[i].folder_no = i;
    ck_data.folders[i].direction = CK_DIRECTION_TRANSMIT;
    ck_data.folders[i].doc_list_no = 0;
    ck_data.folders[i].doc_no = i - 1;  // 0 reserved by mayor's doc
    ck_data.folders[i].enable = true;
    ck_data.folders[i].dlc = 2;
  }
  // NOLINTEND(*-magic-numbers)
}