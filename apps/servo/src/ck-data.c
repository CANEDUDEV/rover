#include "ck-data.h"

#include <stdio.h>
#include <string.h>

#include "error.h"
#include "json.h"
#include "jsondb.h"

static ck_data_t ck_data;

void page_init(void);
void doc_init(void);
void list_init(void);
void folder_init(void);
static void assign_stored(void);

static int check_folder(json_object_t *folder);
static int check_envelope(json_object_t *envelope);
static void assign(json_object_t *folder, json_object_t *envelope);

void ck_data_init(void) {
  page_init();
  doc_init();
  list_init();
  folder_init();
  assign_stored();
}

ck_data_t *get_ck_data(void) {
  return &ck_data;
}

void page_init(void) {
  ck_data.servo_position_page = &ck_data.pages[0];
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
  ck_data.servo_position_folder = &ck_data.folders[2];
  ck_data.servo_current_folder = &ck_data.folders[3];
  ck_data.battery_voltage_folder = &ck_data.folders[4];
  ck_data.servo_voltage_folder = &ck_data.folders[5];
  ck_data.h_bridge_current_folder = &ck_data.folders[6];
  ck_data.set_servo_voltage_folder = &ck_data.folders[7];
  ck_data.pwm_conf_folder = &ck_data.folders[8];
  ck_data.steering_folder = &ck_data.folders[9];
  ck_data.subtrim_folder = &ck_data.folders[10];
  ck_data.report_freq_folder = &ck_data.folders[11];
  ck_data.reverse_folder = &ck_data.folders[12];
  ck_data.failsafe_folder = &ck_data.folders[13];
  // NOLINTEND(*-magic-numbers)

  // Set up the transmit folders
  for (int i = 2; i < 2 + CK_DATA_TX_FOLDER_COUNT; i++) {
    ck_data.folders[i].folder_no = i;
    ck_data.folders[i].direction = CK_DIRECTION_TRANSMIT;
    ck_data.folders[i].doc_list_no = 0;
    ck_data.folders[i].doc_no = i - 1;  // 0 reserved by mayor's doc
    ck_data.folders[i].enable = true;
    ck_data.folders[i].dlc = 2;
  }

  // Set up the receive folders
  for (int i = 2 + CK_DATA_TX_FOLDER_COUNT; i < CK_DATA_FOLDER_COUNT; i++) {
    ck_data.folders[i].folder_no = i;
    ck_data.folders[i].direction = CK_DIRECTION_RECEIVE;
    ck_data.folders[i].doc_list_no = 0;
    ck_data.folders[i].enable = true;
    ck_data.folders[i].doc_no = i - (2 + CK_DATA_TX_FOLDER_COUNT);
  }

  // NOLINTBEGIN(*-magic-numbers)
  ck_data.set_servo_voltage_folder->dlc = sizeof(uint16_t);
  ck_data.pwm_conf_folder->dlc = sizeof(uint16_t);
  ck_data.steering_folder->dlc = sizeof(uint8_t) + sizeof(float);
  ck_data.subtrim_folder->dlc = sizeof(int16_t);
  ck_data.report_freq_folder->dlc = sizeof(uint16_t);
  ck_data.reverse_folder->dlc = 0;
  ck_data.failsafe_folder->dlc = sizeof(uint8_t) + 2 * sizeof(uint16_t);
  // NOLINTEND(*-magic-numbers)
}

static void assign_stored(void) {
  json_object_t *json = get_jsondb();
  json_object_t *assignments = json_get_object("assignments", json);
  if (!assignments || !assignments->child) {
    printf("note: no assignments stored\r\n");
    return;
  }

  json_object_t *assignment = assignments->child;

  do {
    json_object_t *folder = json_get_object("folder", assignment);
    json_object_t *envelope = json_get_object("envelope", assignment);
    assign(folder, envelope);
    assignment = assignment->next;
  } while (assignment != NULL);
}

static int check_folder(json_object_t *folder) {
  if (folder->type != JSON_INT || folder->value->int_ < 2 ||
      folder->value->int_ >= CK_DATA_FOLDER_COUNT) {
    return APP_NOT_OK;
  }
  return APP_OK;
}

static int check_envelope(json_object_t *envelope) {
  if (envelope->type != JSON_INT || envelope->value->int_ < 0) {
    return APP_NOT_OK;
  }

  ck_envelope_t e = {
      .envelope_no = envelope->value->int_,
      .has_extended_id = false,
  };

  if (ck_check_envelope(&e) != CK_OK) {
    return APP_NOT_OK;
  }

  return APP_OK;
}

static void assign(json_object_t *folder, json_object_t *envelope) {
  static int i = 0;
  char s[32];  // NOLINT(*-magic-numbers)
  if (check_folder(folder) != APP_OK) {
    json_sprint(s, folder);
    printf("array element %d: couldn't parse folder: %s\r\n", i++, s);
    return;
  }
  if (check_envelope(envelope) != APP_OK) {
    json_sprint(s, envelope);
    printf("array element %d: couldn't parse envelope: %s\r\n", i++, s);
    return;
  }

  i++;

  ck_folder_t *f = &ck_data.folders[folder->value->int_];
  if (f->envelope_count >= CK_MAX_ENVELOPES_PER_FOLDER) {
    return;
  }

  ck_envelope_t e = {
      .envelope_no = envelope->value->int_,
      .enable = true,
      .is_compressed = false,
      .is_remote = false,
      .has_extended_id = false,
  };

  // Only add envelope if it's not assigned to this folder already
  int envelope_index = ck_find_envelope(f, &e);
  if (envelope_index < 0) {
    f->envelopes[f->envelope_count] = e;
    f->envelope_count++;
  }
}
