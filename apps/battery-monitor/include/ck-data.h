#ifndef CK_DATA_H
#define CK_DATA_H

#ifdef __cplusplus
extern "C" {
#endif

#include "types.h"

#define CK_DATA_TX_PAGE_COUNT 4
#define CK_DATA_TX_DOC_COUNT 3
#define CK_DATA_RX_DOC_COUNT 7
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

  ck_page_t *cell_page0;
  ck_page_t *cell_page1;
  ck_page_t *reg_out_page;
  ck_page_t *vbat_out_page;

  ck_document_t *cell_doc;
  ck_document_t *reg_out_doc;
  ck_document_t *vbat_out_doc;

  // Transmit
  ck_folder_t *cell_folder;
  ck_folder_t *reg_out_folder;
  ck_folder_t *vbat_out_folder;

  // Receive
  ck_folder_t *jumper_config_folder;
  ck_folder_t *vbat_out_overcurrent_threshold_folder;
  ck_folder_t *reg_out_overcurrent_threshold_folder;
  ck_folder_t *set_reg_out_voltage_folder;
  ck_folder_t *output_on_off_folder;
  ck_folder_t *report_freq_folder;
  ck_folder_t *low_voltage_cutoff_folder;

} ck_data_t;

void ck_data_init(void);
ck_data_t *get_ck_data(void);

#ifdef __cplusplus
}
#endif

#endif /* CK_DATA_H */
