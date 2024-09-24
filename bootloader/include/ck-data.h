#ifndef CK_DATA_H
#define CK_DATA_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ck-types.h"

#define CK_DATA_TX_PAGE_COUNT 4
#define CK_DATA_TX_DOC_COUNT 3
#define CK_DATA_RX_DOC_COUNT 6
#define CK_DATA_LIST_COUNT 2
#define CK_DATA_TX_FOLDER_COUNT CK_DATA_TX_DOC_COUNT
#define CK_DATA_RX_FOLDER_COUNT CK_DATA_RX_DOC_COUNT
#define CK_DATA_FOLDER_COUNT \
  (2 + CK_DATA_TX_FOLDER_COUNT + CK_DATA_RX_FOLDER_COUNT)

typedef struct {
  // Don't need to store receive docs and receive pages.
  ck_page_t pages[CK_DATA_TX_PAGE_COUNT];
  ck_document_t docs[CK_DATA_TX_DOC_COUNT];
  ck_list_t lists[CK_DATA_LIST_COUNT];
  ck_folder_t folders[CK_DATA_FOLDER_COUNT];

  // Convenience pointers
  ck_list_t *tx_list;
  ck_list_t *rx_list;

  ck_page_t *bootloader_page;  // Added as a mayor's page. Used to let king ask
                               // node if it's inside its bootloader. The page
                               // is length 8 and filled with zeroes.
  ck_page_t *command_ack_page;

  // Block transfer pages
  ck_page_t *bundle_request_page;
  ck_page_t *abort_page;

  ck_document_t *command_ack_doc;
  ck_document_t *program_doc;
  ck_document_t *config_doc;

  // Transmit
  ck_folder_t *command_ack_folder;
  ck_folder_t *program_transmit_folder;  // For abort page
  ck_folder_t *config_transmit_folder;   // For abort page

  // Receive
  ck_folder_t *enter_bootloader_folder;  // Don't jump to app
  ck_folder_t *exit_bootloader_folder;   // Jump to app
  ck_folder_t *flash_erase_folder;       // Erase flash
  ck_folder_t *fs_format_folder;         // Reformaat  filesystem
  ck_folder_t *program_receive_folder;   // Receive program data
  ck_folder_t *config_receive_folder;    // Receive config data

} ck_data_t;

void ck_data_init(void);
ck_data_t *get_ck_data(void);

#ifdef __cplusplus
}
#endif

#endif /* CK_DATA_H */
