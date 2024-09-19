#ifndef CK_DATA_H
#define CK_DATA_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ck-types.h"

#define CK_DATA_TX_PAGE_COUNT 5
#define CK_DATA_TX_DOC_COUNT 5
#define CK_DATA_LIST_COUNT 2
#define CK_DATA_TX_FOLDER_COUNT 5
#define CK_DATA_RX_FOLDER_COUNT 7
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

  ck_page_t *servo_position_page;
  ck_page_t *servo_current_page;
  ck_page_t *battery_voltage_page;
  ck_page_t *servo_voltage_page;
  ck_page_t *h_bridge_current_page;

  // Transmit
  ck_folder_t *servo_position_folder;
  ck_folder_t *servo_current_folder;
  ck_folder_t *battery_voltage_folder;
  ck_folder_t *servo_voltage_folder;
  ck_folder_t *h_bridge_current_folder;

  // Receive
  ck_folder_t *set_servo_voltage_folder;
  ck_folder_t *pwm_conf_folder;
  ck_folder_t *steering_folder;
  ck_folder_t *subtrim_folder;
  ck_folder_t *report_freq_folder;
  ck_folder_t *reverse_folder;
  ck_folder_t *failsafe_folder;

} ck_data_t;

void ck_data_init(void);
ck_data_t *get_ck_data(void);

#ifdef __cplusplus
}
#endif

#endif /* CK_DATA_H */
