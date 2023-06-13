
#ifndef CK_DATA_H
#define CK_DATA_H

#ifdef __cplusplus
extern "C" {
#endif

#include "types.h"

#define CK_DATA_PAGE_COUNT 4
#define CK_DATA_DOC_COUNT 3
#define CK_DATA_LIST_COUNT 2
#define CK_DATA_FOLDER_COUNT 5

typedef struct {
  ck_page_t pages[CK_DATA_PAGE_COUNT];
  ck_document_t docs[CK_DATA_DOC_COUNT];
  ck_list_t lists[CK_DATA_LIST_COUNT];
  ck_folder_t folders[CK_DATA_FOLDER_COUNT];

  // Convenience pointers
  ck_list_t *tx_list;
  ck_list_t *rx_list;

  ck_page_t *cell_page0;
  ck_page_t *cell_page1;
  ck_page_t *reg_out_current_page;
  ck_page_t *vbat_out_current_page;

  ck_document_t *cell_doc;
  ck_document_t *reg_out_current_doc;
  ck_document_t *vbat_out_current_doc;

  ck_folder_t *cell_folder;
  ck_folder_t *reg_out_current_folder;
  ck_folder_t *vbat_out_current_folder;

} ck_data_t;

void ck_data_init(void);
ck_data_t *get_ck_data(void);

#ifdef __cplusplus
}
#endif

#endif /* CK_DATA_H */
