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
  ck_data.cell_page0 = &ck_data.pages[0];
  ck_data.cell_page1 = &ck_data.pages[1];
  ck_data.reg_out_current_page = &ck_data.pages[2];
  ck_data.vbat_out_current_page = &ck_data.pages[3];

  // Set up the pages
  // NOLINTBEGIN(*-magic-numbers)
  ck_data.cell_page0->line_count = 7;
  ck_data.cell_page0->lines[0] = 0;  // Pagination
  ck_data.cell_page1->line_count = 7;
  ck_data.cell_page1->lines[0] = 1;  // Pagination
  ck_data.reg_out_current_page->line_count = 2;
  ck_data.vbat_out_current_page->line_count = 4;
  // NOLINTEND(*-magic-numbers)
}

void doc_init(void) {
  ck_data.cell_doc = &ck_data.docs[0];
  ck_data.reg_out_current_doc = &ck_data.docs[1];
  ck_data.vbat_out_current_doc = &ck_data.docs[2];

  // Set up the documents
  ck_data.cell_doc->direction = CK_DIRECTION_TRANSMIT;
  ck_data.cell_doc->page_count = 2;
  ck_data.cell_doc->pages[0] = ck_data.cell_page0;
  ck_data.cell_doc->pages[1] = ck_data.cell_page1;

  ck_data.reg_out_current_doc->direction = CK_DIRECTION_TRANSMIT;
  ck_data.reg_out_current_doc->page_count = 1;
  ck_data.reg_out_current_doc->pages[0] = ck_data.reg_out_current_page;

  ck_data.vbat_out_current_doc->direction = CK_DIRECTION_TRANSMIT;
  ck_data.vbat_out_current_doc->page_count = 1;
  ck_data.vbat_out_current_doc->pages[0] = ck_data.vbat_out_current_page;
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

  // We have 3 documents, and CK needs 1 slot for the mayor's doc.
  ck_data.tx_list->record_count = 4;
  ck_data.tx_list->records[1] = ck_data.cell_doc;
  ck_data.tx_list->records[2] = ck_data.reg_out_current_doc;
  ck_data.tx_list->records[3] = ck_data.vbat_out_current_doc;
}

void folder_init(void) {
  // NOLINTBEGIN(*-magic-numbers)
  ck_data.cell_folder = &ck_data.folders[2];
  ck_data.reg_out_current_folder = &ck_data.folders[3];
  ck_data.vbat_out_current_folder = &ck_data.folders[4];
  ck_data.jumper_and_fuse_conf_folder = &ck_data.folders[5];
  ck_data.reg_out_voltage_folder = &ck_data.folders[6];
  ck_data.output_on_off_folder = &ck_data.folders[7];
  ck_data.report_freq_folder = &ck_data.folders[8];
  ck_data.low_voltage_cutoff_folder = &ck_data.folders[9];

  // Set up the transmit folders
  for (int i = 2; i < 5; i++) {
    ck_data.folders[i].folder_no = i;
    ck_data.folders[i].direction = CK_DIRECTION_TRANSMIT;
    ck_data.folders[i].doc_list_no = 0;
    ck_data.folders[i].doc_no = i - 1;  // 0 reserved by mayor's doc
    ck_data.folders[i].enable = true;
  }

  // 3 cells per page + 1 byte for pagination.
  ck_data.cell_folder->dlc = 3 * sizeof(uint16_t) + 1;
  ck_data.reg_out_current_folder->dlc = sizeof(uint16_t);
  ck_data.vbat_out_current_folder->dlc = sizeof(uint32_t);

  // Set up the receive folders
  for (int i = 5; i < CK_DATA_FOLDER_COUNT; i++) {
    ck_data.folders[i].folder_no = i;
    ck_data.folders[i].direction = CK_DIRECTION_RECEIVE;
    ck_data.folders[i].doc_list_no = 1;
    ck_data.folders[i].doc_no = i - 5;
    ck_data.folders[i].enable = true;
  }

  ck_data.jumper_and_fuse_conf_folder->dlc =
      2 * sizeof(uint8_t) + sizeof(uint32_t);
  ck_data.reg_out_voltage_folder->dlc = sizeof(uint16_t);
  ck_data.output_on_off_folder->dlc = 2 * sizeof(uint8_t);
  ck_data.report_freq_folder->dlc = sizeof(uint32_t);
  ck_data.low_voltage_cutoff_folder->dlc = sizeof(uint16_t);
  // NOLINTEND(*-magic-numbers)
}
