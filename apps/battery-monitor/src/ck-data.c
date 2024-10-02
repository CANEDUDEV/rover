#include "ck-data.h"

#include "error.h"
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
  ck_data.cell_page0 = &ck_data.pages[0];
  ck_data.cell_page1 = &ck_data.pages[1];
  ck_data.reg_out_page = &ck_data.pages[2];
  ck_data.vbat_out_page = &ck_data.pages[3];

  // 3 cells per page + 1 byte for pagination.
  ck_data.cell_page0->line_count = 3 * sizeof(uint16_t) + 1;
  ck_data.cell_page0->lines[0] = 0;  // Pagination
  ck_data.cell_page1->line_count = 3 * sizeof(uint16_t) + 1;
  ck_data.cell_page1->lines[0] = 1;  // Pagination
  // Voltage and current are uin32_t.
  ck_data.reg_out_page->line_count = 2 * sizeof(uint32_t);
  ck_data.vbat_out_page->line_count = 2 * sizeof(uint32_t);
}

void doc_init(void) {
  ck_data.cell_doc = &ck_data.docs[0];
  ck_data.reg_out_doc = &ck_data.docs[1];
  ck_data.vbat_out_doc = &ck_data.docs[2];

  // Set up the documents
  ck_data.cell_doc->direction = CK_DIRECTION_TRANSMIT;
  ck_data.cell_doc->page_count = 2;
  ck_data.cell_doc->pages[0] = ck_data.cell_page0;
  ck_data.cell_doc->pages[1] = ck_data.cell_page1;

  ck_data.reg_out_doc->direction = CK_DIRECTION_TRANSMIT;
  ck_data.reg_out_doc->page_count = 1;
  ck_data.reg_out_doc->pages[0] = ck_data.reg_out_page;

  ck_data.vbat_out_doc->direction = CK_DIRECTION_TRANSMIT;
  ck_data.vbat_out_doc->page_count = 1;
  ck_data.vbat_out_doc->pages[0] = ck_data.vbat_out_page;
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
  ck_data.tx_list->records[1] = ck_data.cell_doc;
  ck_data.tx_list->records[2] = ck_data.reg_out_doc;
  ck_data.tx_list->records[3] = ck_data.vbat_out_doc;
}

void folder_init(void) {
  // NOLINTBEGIN(*-magic-numbers)
  ck_data.cell_folder = &ck_data.folders[2];
  ck_data.reg_out_folder = &ck_data.folders[3];
  ck_data.vbat_out_folder = &ck_data.folders[4];
  ck_data.jumper_config_folder = &ck_data.folders[5];
  ck_data.set_reg_out_voltage_folder = &ck_data.folders[6];
  ck_data.output_on_off_folder = &ck_data.folders[7];
  ck_data.report_freq_folder = &ck_data.folders[8];
  ck_data.low_voltage_cutoff_folder = &ck_data.folders[9];
  ck_data.vbat_out_overcurrent_threshold_folder = &ck_data.folders[10];
  ck_data.reg_out_overcurrent_threshold_folder = &ck_data.folders[11];
  // NOLINTEND(*-magic-numbers)

  // Set up the transmit folders
  for (int i = 2; i < 2 + CK_DATA_TX_FOLDER_COUNT; i++) {
    ck_data.folders[i].folder_no = i;
    ck_data.folders[i].direction = CK_DIRECTION_TRANSMIT;
    ck_data.folders[i].doc_list_no = 0;
    ck_data.folders[i].doc_no = i - 1;  // 0 reserved by mayor's doc
    ck_data.folders[i].enable = true;
  }

  ck_data.cell_folder->dlc = ck_data.cell_page0->line_count;
  ck_data.reg_out_folder->dlc = ck_data.reg_out_page->line_count;
  ck_data.vbat_out_folder->dlc = ck_data.vbat_out_page->line_count;

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

  ck_data.jumper_config_folder->dlc = sizeof(uint8_t);
  ck_data.set_reg_out_voltage_folder->dlc = sizeof(uint32_t);
  ck_data.output_on_off_folder->dlc = 2 * sizeof(uint8_t);
  ck_data.report_freq_folder->dlc = sizeof(uint16_t);
  ck_data.low_voltage_cutoff_folder->dlc = sizeof(uint16_t);
  ck_data.vbat_out_overcurrent_threshold_folder->dlc = sizeof(uint32_t);
  ck_data.reg_out_overcurrent_threshold_folder->dlc = sizeof(uint32_t);
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
