#ifndef CK_DATA_H
#define CK_DATA_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ck-types.h"

#define CK_DATA_TX_PAGE_COUNT 1
#define CK_DATA_TX_DOC_COUNT 1
#define CK_DATA_LIST_COUNT 2
#define CK_DATA_TX_FOLDER_COUNT 1
#define CK_DATA_RX_FOLDER_COUNT 2
#define CK_DATA_FOLDER_COUNT \
  (2 + CK_DATA_TX_FOLDER_COUNT + CK_DATA_RX_FOLDER_COUNT)

typedef struct {
  ck_page_t pages[CK_DATA_TX_PAGE_COUNT];
  ck_document_t docs[CK_DATA_TX_DOC_COUNT];
  ck_list_t lists[CK_DATA_LIST_COUNT];
  ck_folder_t folders[CK_DATA_FOLDER_COUNT];

  // Convenience pointers
  ck_list_t *tx_list;
  ck_list_t *rx_list;

  ck_page_t *wheel_speed_page;

  // Transmit
  ck_folder_t *wheel_speed_folder;

  // Receive
  ck_folder_t *set_wheel_parameters_folder;
  ck_folder_t *set_report_freq_folder;

} ck_data_t;

void ck_data_init(void);
ck_data_t *get_ck_data(void);

#ifdef __cplusplus
}
#endif

#endif /* CK_DATA_H */
