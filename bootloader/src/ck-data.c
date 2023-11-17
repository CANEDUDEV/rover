#include "ck-data.h"

#include <string.h>

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
  ck_data.bootloader_page = &ck_data.pages[0];
  // Needs to have DLC=8 since it's a mayor's page
  ck_data.bootloader_page->line_count = CK_MAX_LINES_PER_PAGE;
  memset(ck_data.bootloader_page, 0, ck_data.bootloader_page->line_count);
  ck_data.bootloader_page->lines[1] = 2;  // Page number 2

  // Contains a CAN ID
  ck_data.command_ack_page = &ck_data.pages[1];

  // NOLINTBEGIN(*-magic-numbers)
  // 1 byte for ACK/NACK, 4 bytes with ID
  ck_data.command_ack_page->line_count = 5;

  // The "block transfer page 2" as specified in CK.
  ck_data.flash_program_bundle_request_page = &ck_data.pages[2];
  ck_data.flash_program_bundle_request_page->line_count = 8;
  ck_data.flash_program_bundle_request_page->lines[0] = 2;

  // The "block transfer page 5" as specified in CK.
  ck_data.flash_program_abort_page = &ck_data.pages[3];
  ck_data.flash_program_abort_page->line_count = 8;
  ck_data.flash_program_abort_page->lines[0] = 5;
  // NOLINTEND(*-magic-numbers)
}

void doc_init(void) {
  // Don't add bootloader_page since it will be added as a mayor's page.
  ck_data.command_ack_doc = &ck_data.docs[0];
  ck_data.command_ack_doc->direction = CK_DIRECTION_TRANSMIT;
  ck_data.command_ack_doc->page_count = 1;
  ck_data.command_ack_doc->pages[0] = ck_data.command_ack_page;

  ck_data.flash_program_doc = &ck_data.docs[1];
  ck_data.flash_program_doc->direction = CK_DIRECTION_TRANSMIT;

  // This document is special since some pages are transmit pages and others are
  // receive. We only implement the transmit pages here.
  //
  // NOLINTBEGIN(*-magic-numbers)
  ck_data.flash_program_doc->page_count = 6;
  ck_data.flash_program_doc->pages[2] =
      ck_data.flash_program_bundle_request_page;
  ck_data.flash_program_doc->pages[5] = ck_data.flash_program_abort_page;
  // NOLINTEND(*-magic-numbers)
}

void list_init(void) {
  ck_data.tx_list = &ck_data.lists[0];
  ck_data.rx_list = &ck_data.lists[1];

  // Set up the doc lists
  ck_data.rx_list->type = CK_LIST_DOCUMENT;
  ck_data.rx_list->direction = CK_DIRECTION_RECEIVE;
  ck_data.rx_list->list_no = 0;
  ck_data.rx_list->record_count =
      CK_DATA_RX_DOC_COUNT + 1;  // 1 slot reserved for the king's doc.

  ck_data.tx_list->type = CK_LIST_DOCUMENT;
  ck_data.tx_list->direction = CK_DIRECTION_TRANSMIT;
  ck_data.tx_list->list_no = 0;

  // CK needs 1 slot for the mayor's doc.
  ck_data.tx_list->record_count = CK_DATA_TX_DOC_COUNT + 1;
  ck_data.tx_list->records[1] = ck_data.command_ack_doc;
  ck_data.tx_list->records[2] = ck_data.flash_program_doc;
}

void folder_init(void) {
  // NOLINTBEGIN(*-magic-numbers)
  ck_data.command_ack_folder = &ck_data.folders[2];
  ck_data.flash_program_transmit_folder = &ck_data.folders[3];
  ck_data.enter_bootloader_folder = &ck_data.folders[4];
  ck_data.exit_bootloader_folder = &ck_data.folders[5];
  ck_data.flash_erase_folder = &ck_data.folders[6];
  ck_data.flash_program_receive_folder = &ck_data.folders[7];
  // NOLINTEND(*-magic-numbers)

  // Set up the transmit folders
  for (int i = 2; i < 2 + CK_DATA_TX_FOLDER_COUNT; i++) {
    ck_data.folders[i].folder_no = i;
    ck_data.folders[i].direction = CK_DIRECTION_TRANSMIT;
    ck_data.folders[i].doc_list_no = 0;
    ck_data.folders[i].doc_no = i - 1;  // 0 reserved by mayor's doc
    ck_data.folders[i].enable = true;
  }

  // NOLINTBEGIN(*-magic-numbers)
  ck_data.command_ack_folder->dlc = 5;
  ck_data.flash_program_transmit_folder->dlc = 8;
  // NOLINTEND(*-magic-numbers)

  // Set up the receive folders
  uint8_t rx_doc_no = 0;  // Start counting from 0
  for (int i = 2 + CK_DATA_TX_FOLDER_COUNT; i < CK_DATA_FOLDER_COUNT; i++) {
    ck_data.folders[i].folder_no = i;
    ck_data.folders[i].direction = CK_DIRECTION_RECEIVE;
    ck_data.folders[i].doc_list_no = 1;
    ck_data.folders[i].doc_no = rx_doc_no;
    ck_data.folders[i].enable = true;

    rx_doc_no++;
  }

  // NOLINTBEGIN(*-magic-numbers)
  ck_data.enter_bootloader_folder->dlc = 0;
  ck_data.exit_bootloader_folder->dlc = 0;
  ck_data.flash_erase_folder->dlc = 4;
  ck_data.flash_program_receive_folder->dlc = 8;
  // NOLINTEND(*-magic-numbers)
}
