#include "mayor.h"

#include <string.h>

struct mayor_state {
  ck_mayor_t user_data;  // Data provided by the user.
  ck_page_t pages[2];    // For storing the predefined mayor's pages.
  ck_document_t mayors_doc;
  ck_document_t kings_doc;  // Dummy king's document.
};

static struct mayor_state mayor;  // Holds the mayor's state.

// Helpers
static void init_mayors_pages(void);
static void init_documents(void);
static void init_folders(void);
static ck_err_t init_lists(void);
static ck_folder_t kings_folder(void);
static ck_folder_t mayors_folder(void);

static ck_err_t process_kp0(const ck_page_t *page);
static ck_err_t process_kp1(const ck_page_t *page);
static ck_err_t process_kp2(const ck_page_t *page);
static ck_err_t process_kp16(const ck_page_t *page);
static ck_err_t process_kp17(const ck_page_t *page);

static ck_err_t remove_envelope_from_folder(ck_folder_t *folder,
                                            const ck_envelope_t *envelope);
static ck_err_t assign_envelope(uint8_t folder_no,
                                const ck_envelope_t *envelope);
static ck_err_t expel_envelope(const ck_envelope_t *envelope);
static ck_err_t transfer_envelope(uint8_t folder_no,
                                  const ck_envelope_t *envelope);

// Returns true if mayor is in group with specified group number, false
// otherwise.
static bool in_group(uint8_t group_no);

// These return the item index if found, -1 otherwise.
static int find_folder(uint8_t folder_no);
static int find_envelope(const ck_folder_t *folder,
                         const ck_envelope_t *envelope);

// Returns pointer to list if found, NULL otherwise.
static ck_list_t *find_list(ck_list_type_t list_type, ck_direction_t direction,
                            uint8_t list_no);

// Set the source list type given a target list type.
static ck_err_t get_source_list_type(ck_list_type_t target_list_type,
                                     ck_list_type_t *source_list_type);

ck_err_t ck_mayor_init(const ck_mayor_t *mayor_) {
  // Check for unset parameters.
  if (mayor_->ean_no == 0 || mayor_->serial_no == 0 ||
      mayor_->city_address == 0 || !mayor_->set_action_mode ||
      !mayor_->set_comm_mode || !mayor_->set_city_mode || !mayor_->folders) {
    return CK_ERR_MISSING_PARAMETER;
  }

  // Verify EAN and serial numbers are at most 40 bits.
  if (mayor_->ean_no > CK_MAX_EAN_NO || mayor_->serial_no > CK_MAX_EAN_NO) {
    return CK_ERR_INVALID_PARAMETER;
  }

  // Need at least 2 folders and 2 lists.
  if (mayor_->folder_count < 2 || mayor_->list_count < 2) {
    return CK_ERR_INVALID_PARAMETER;
  }

  // Copy user data to internal state.
  // Needs to be done before setting up the internal state.
  memcpy(&mayor.user_data, mayor_, sizeof(ck_mayor_t));

  init_mayors_pages();
  init_documents();
  init_folders();
  return init_lists();
}

ck_err_t ck_process_kings_letter(const ck_letter_t *letter) {
  // Check if mayor has been initialized
  if (mayor.user_data.city_address == 0) {
    return CK_ERR_NOT_INITIALIZED;
  }

  // All king's pages have 8 lines.
  if (letter->page.line_count != CK_MAX_LINES_PER_PAGE) {
    return CK_ERR_INVALID_KINGS_LETTER;
  }
  // King's letter shouldn't have the RTR bit set.
  if (letter->envelope.is_remote) {
    return CK_ERR_INVALID_KINGS_LETTER;
  }

  uint8_t address = letter->page.lines[0];

  // Check if letter is addressed to us.
  if (address != 0 && address != mayor.user_data.city_address &&
      !in_group(address)) {
    return CK_OK;
  }

  uint8_t page_no = letter->page.lines[1];

  switch (page_no) {
    case CK_KP0:
      return process_kp0(&letter->page);
    case CK_KP1:
      return process_kp1(&letter->page);
    case CK_KP2:
      return process_kp2(&letter->page);
    case CK_KP16:
      return process_kp16(&letter->page);
    case CK_KP17:
      return process_kp17(&letter->page);
    default:
      return CK_ERR_UNSUPPORTED_KINGS_PAGE;
  }
}

ck_err_t ck_add_mayors_page(ck_page_t *page) {
  if (!page) {
    return CK_ERR_INVALID_PARAMETER;
  }

  if (mayor.mayors_doc.page_count >= CK_MAX_PAGES_PER_DOCUMENT) {
    return CK_ERR_CAPACITY_REACHED;
  }

  mayor.mayors_doc.pages[mayor.mayors_doc.page_count] = page;
  mayor.mayors_doc.page_count++;

  return CK_OK;
}

static void init_mayors_pages(void) {
  // Init mayor's pages
  mayor.pages[0].line_count = CK_MAX_LINES_PER_PAGE;
  mayor.pages[1].line_count = CK_MAX_LINES_PER_PAGE;
  mayor.pages[0].lines[1] = 0;  // Page 0
  mayor.pages[1].lines[1] = 1;  // Page 1

  // EAN no and serial no are 40 bits long (5 bytes)
  // NOLINTBEGIN(*-magic-numbers)
  memcpy(&mayor.pages[0].lines[2], &mayor.user_data.ean_no, 5);
  memcpy(&mayor.pages[1].lines[2], &mayor.user_data.serial_no, 5);
  // NOLINTEND(*-magic-numbers)
}

static void init_documents(void) {
  // Init mayor's document
  mayor.mayors_doc.direction = CK_DIRECTION_TRANSMIT;
  mayor.mayors_doc.page_count = 2;

  // Point mayor's document to the predefined mayor's pages
  mayor.mayors_doc.pages[0] = &mayor.pages[0];
  mayor.mayors_doc.pages[1] = &mayor.pages[1];

  // The king's folder needs to point to something.
  // This dummy king's document will suffice.
  mayor.kings_doc.direction = CK_DIRECTION_RECEIVE;
}

static void init_folders(void) {
  // Init mayor's folders
  mayor.user_data.folders[0] = kings_folder();
  mayor.user_data.folders[1] = mayors_folder();
}

static ck_err_t init_lists(void) {
  // Transmit document list 0 and receive document list 0 must exist.
  ck_list_t *rx_list = find_list(CK_LIST_DOCUMENT, CK_DIRECTION_RECEIVE, 0);
  if (!rx_list) {
    return CK_ERR_ITEM_NOT_FOUND;
  }
  ck_list_t *tx_list = find_list(CK_LIST_DOCUMENT, CK_DIRECTION_TRANSMIT, 0);
  if (!tx_list) {
    return CK_ERR_ITEM_NOT_FOUND;
  }

  // Need at least one record in the list to put the mayor's document in it
  if (tx_list->capacity < 1 || rx_list->capacity < 1) {
    return CK_ERR_CAPACITY_REACHED;
  }

  // Set up the mayor's document in the document transmit list
  tx_list->records[0] = &mayor.mayors_doc;
  rx_list->records[0] = &mayor.kings_doc;

  return CK_OK;
}

static ck_err_t process_kp0(const ck_page_t *page) {
  ck_action_mode_t action_mode = page->lines[2];
  ck_err_t ret = ck_check_action_mode(action_mode);
  if (ret != CK_OK) {
    return CK_ERR_INVALID_KINGS_LETTER;
  }

  ret = mayor.user_data.set_action_mode(action_mode);
  if (ret != CK_OK) {
    return ret;
  }

  ck_comm_mode_t comm_mode = page->lines[3];
  ret = ck_check_comm_mode(comm_mode);
  if (ret != CK_OK) {
    return CK_ERR_INVALID_KINGS_LETTER;
  }

  ret = mayor.user_data.set_comm_mode(comm_mode);
  if (ret != CK_OK) {
    return ret;
  }

  // Mayor will have to handle invalid city modes themselves.
  ck_city_mode_t city_mode = page->lines[4];
  return mayor.user_data.set_city_mode(city_mode);
}

static ck_err_t process_kp1(const ck_page_t *page) {
  uint32_t base_no = 0;
  memcpy(&base_no, &page->lines[4], sizeof(base_no));
  base_no &= CK_CAN_ID_MASK;
  bool extended_id = (page->lines[7] >> 7) & 0x01;  // NOLINT(*-magic-numbers)

  // Bounds check
  if ((!extended_id &&
       base_no + mayor.user_data.city_address > CK_CAN_MAX_STD_ID) ||
      (extended_id &&
       base_no + mayor.user_data.city_address > CK_CAN_MAX_EXT_ID)) {
    return CK_ERR_INVALID_CAN_ID;
  }

  mayor.user_data.base_no = base_no;
  mayor.user_data.has_extended_id = extended_id;

  mayor.user_data.folders[1].envelopes[0].envelope_no =
      mayor.user_data.base_no + mayor.user_data.city_address;

  if (page->lines[2] != CK_NO_RESPONSE_REQUESTED) {
    // TODO: Implement responding with mayor's document
    // return send_mayors_doc(page->line[2]);
  }

  return CK_OK;
}

static ck_err_t process_kp2(const ck_page_t *page) {
  // NOLINTBEGIN(*-magic-numbers)
  ck_envelope_t envelope;
  memcpy(&envelope.envelope_no, &page->lines[2], sizeof(envelope.envelope_no));
  envelope.envelope_no &= CK_CAN_ID_MASK;
  envelope.has_extended_id = (page->lines[5] >> 7) & 0x01;
  envelope.is_compressed = (page->lines[5] >> 6) & 0x01;

  uint8_t folder_no = page->lines[6];
  envelope.enable = page->lines[7] & 0x01;
  ck_envelope_action_t envelope_action = (page->lines[7] >> 1) & 0x03;
  // NOLINTEND(*-magic-numbers)

  // Bounds check
  if (!envelope.has_extended_id && envelope.envelope_no > CK_CAN_MAX_STD_ID) {
    return CK_ERR_INVALID_CAN_ID;
  }

  switch (envelope_action) {
    case CK_ENVELOPE_NO_ACTION:
      return CK_OK;

    case CK_ENVELOPE_ASSIGN:
      return assign_envelope(folder_no, &envelope);

    case CK_ENVELOPE_EXPEL:
      return expel_envelope(&envelope);

    case CK_ENVELOPE_TRANSFER:
      return transfer_envelope(folder_no, &envelope);

    default:
      return CK_ERR_INVALID_KINGS_LETTER;
  }
}

static ck_err_t process_kp16(const ck_page_t *page) {
  uint8_t folder_no = page->lines[2];
  // Folder numbers 0 and 1 are reserved
  if (folder_no < 2) {
    return CK_ERR_INVALID_FOLDER_NUMBER;
  }
  int folder_index = find_folder(folder_no);
  if (folder_index < 0) {
    return CK_ERR_ITEM_NOT_FOUND;
  }

  // NOLINTBEGIN(*-magic-numbers)
  // If all bits in line 3 are 0, then the settings on that line remain
  // unchanged.
  ck_folder_t *folder = &mayor.user_data.folders[folder_index];
  if (page->lines[3] != 0) {
    folder->dlc = page->lines[3] & 0x0F;
    folder->has_rtr = (page->lines[3] >> 6) & 0x01;
    for (uint8_t i = 0; i < folder->envelope_count; i++) {
      folder->envelopes[i].is_remote = folder->has_rtr;
    }
  }

  // If all bits in line 4 are 0, then the settings on that line remain
  // unchanged.
  if (page->lines[4] == 0) {
    return CK_OK;
  }

  folder->direction = page->lines[4] & 0x01;
  folder->enable = (page->lines[4] >> 6) & 0x01;

  ck_document_action_t action = (page->lines[4] >> 4) & 0x03;  // Two bits

  switch (action) {
    case CK_DOCUMENT_NO_ACTION:
      break;

    case CK_DOCUMENT_REMOVE:  // Effectively disables folder.
      folder->doc_no = 0;
      folder->doc_list_no = 0;
      folder->enable = false;
      break;

    case CK_DOCUMENT_INSERT:
      folder->doc_list_no = page->lines[5];
      folder->doc_no = page->lines[6];
      break;

    default:
      return CK_ERR_INVALID_KINGS_LETTER;
  }
  return CK_OK;
  // NOLINTEND(*-magic-numbers)
}

static ck_err_t process_kp17(const ck_page_t *page) {
  // Parse the page
  // NOLINTBEGIN(*-magic-numbers)
  ck_list_type_t target_list_type = (page->lines[2] >> 3) & 0x3;  // Two bits
  ck_direction_t direction = page->lines[2] & 0x01;
  uint8_t source_list_no = page->lines[3];
  uint8_t source_record_no = page->lines[4];
  uint8_t target_list_no = page->lines[5];
  uint8_t target_record_no = page->lines[6];
  uint8_t target_position = page->lines[7];
  // NOLINTEND(*-magic-numbers)

  ck_err_t ret = ck_check_list_type(target_list_type);
  if (ret != CK_OK) {
    return ret;
  }

  ck_list_type_t source_list_type = CK_LIST_BIT;
  ret = get_source_list_type(target_list_type, &source_list_type);
  if (ret != CK_OK) {
    return ret;
  }

  // Find the lists
  ck_list_t *source_list =
      find_list(source_list_type, direction, source_list_no);
  ck_list_t *target_list =
      find_list(target_list_type, direction, target_list_no);

  if (!source_list || !target_list) {
    return CK_ERR_ITEM_NOT_FOUND;
  }

  // Record number bounds check
  if (source_record_no >= source_list->capacity) {
    return CK_ERR_CAPACITY_REACHED;
  }
  if (target_record_no >= target_list->capacity) {
    return CK_ERR_CAPACITY_REACHED;
  }

  // Create the new record
  switch (target_list_type) {
    case CK_LIST_LINE: {
      // Bounds check. A bit's highest position is 7 (position is 0-indexed).
      if (target_position > 7) {  // NOLINT(*-magic-numbers)
        return CK_ERR_INVALID_PARAMETER;
      }

      // Record number in bit list is the bit number, but the elements in the
      // list are bytes. First we extract the byte from the list, then the bit
      // from the byte.
      const uint8_t source_byte_no = source_record_no / 8;
      const uint8_t bit_pos_in_byte = source_record_no % 8;

      uint8_t *source_byte =
          ((uint8_t **)(source_list->records))[source_byte_no];
      uint8_t source_bit = (*source_byte >> bit_pos_in_byte) & 0x01;  // 1 or 0

      uint8_t *target_line =
          ((uint8_t **)(target_list->records))[target_record_no];

      // First clears the bit at the target position, then set it to source_bit.
      *target_line &= ~(1 << target_position);
      *target_line |= source_bit << target_position;

    } break;

    case CK_LIST_PAGE: {
      // Bounds check (position is 0-indexed).
      if (target_position >= CK_MAX_LINES_PER_PAGE) {
        return CK_ERR_INVALID_PARAMETER;
      }

      uint8_t *source_line =
          ((uint8_t **)(source_list->records))[source_record_no];
      ck_page_t *target_page =
          ((ck_page_t **)(target_list->records))[target_record_no];

      // Increase size of page if needed.
      if (target_position >= target_page->line_count) {
        target_page->line_count = target_position + 1;
      }

      target_page->lines[target_position] = *source_line;

    } break;

    case CK_LIST_DOCUMENT: {
      // Bounds check (position is 0-indexed).
      if (target_position >= CK_MAX_PAGES_PER_DOCUMENT) {
        return CK_ERR_INVALID_PARAMETER;
      }

      ck_page_t *source_page =
          ((ck_page_t **)(source_list->records))[source_record_no];
      ck_document_t *target_doc =
          ((ck_document_t **)(target_list->records))[target_record_no];

      target_doc->direction = direction;

      // Increase size of document if needed.
      if (target_position >= target_doc->page_count) {
        target_doc->page_count = target_position + 1;
      }

      target_doc->pages[target_position] = source_page;

    } break;

    default:
      return CK_ERR_INVALID_KINGS_LETTER;
  }

  return CK_OK;
}

static ck_folder_t kings_folder(void) {
  ck_folder_t folder;
  folder.folder_no = 0;
  folder.doc_list_no = 0;
  folder.doc_no = 0;
  folder.direction = CK_DIRECTION_RECEIVE;
  folder.dlc = CK_CAN_MAX_DLC;
  folder.has_rtr = false;
  folder.enable = true;
  folder.envelope_count = 1;
  folder.envelopes[0].envelope_no = 0;
  folder.envelopes[0].enable = true;
  return folder;
}

static ck_folder_t mayors_folder(void) {
  ck_folder_t folder;
  folder.folder_no = 1;
  folder.doc_list_no = 0;
  folder.doc_no = 0;
  folder.direction = CK_DIRECTION_TRANSMIT;
  folder.dlc = CK_CAN_MAX_DLC;
  folder.has_rtr = false;
  folder.enable = true;
  folder.envelope_count = 1;
  folder.envelopes[0].envelope_no =
      mayor.user_data.base_no + mayor.user_data.city_address;
  folder.envelopes[0].enable = true;
  return folder;
}

static bool in_group(uint8_t group_no) {
  for (uint8_t i = 0; i < CK_MAX_GROUPS_PER_CITY; i++) {
    if (mayor.user_data.group_addresses[i] == group_no) {
      return true;
    }
  }
  return false;
}

static int find_folder(uint8_t folder_no) {
  for (uint8_t i = 0; i < mayor.user_data.folder_count; i++) {
    if (mayor.user_data.folders[i].folder_no == folder_no) {
      return i;
    }
  }
  return -1;
}

static int find_envelope(const ck_folder_t *folder,
                         const ck_envelope_t *envelope) {
  for (uint8_t i = 0; i < folder->envelope_count; i++) {
    if (folder->envelopes[i].envelope_no == envelope->envelope_no) {
      return i;
    }
  }
  return -1;
}

static ck_err_t assign_envelope(uint8_t folder_no,
                                const ck_envelope_t *envelope) {
  int folder_index = find_folder(folder_no);
  if (folder_index < 0) {
    return CK_ERR_ITEM_NOT_FOUND;
  }

  ck_folder_t *folder = &mayor.user_data.folders[folder_index];
  if (folder->envelope_count >= CK_MAX_ENVELOPES_PER_FOLDER) {
    return CK_ERR_CAPACITY_REACHED;
  }

  folder->envelopes[folder->envelope_count] = *envelope;
  folder->envelope_count++;

  return CK_OK;
}

static ck_err_t remove_envelope_from_folder(ck_folder_t *folder,
                                            const ck_envelope_t *envelope) {
  int envelope_index = find_envelope(folder, envelope);
  if (envelope_index < 0) {
    return -1;
  }

  // We need to shift all envelopes one step to the left in the array to keep
  // the memory contiguous. This is a noop if the envelope is the last in the
  // array, since in that case the amount of bytes to be moved is 0.
  memmove(
      &folder->envelopes[envelope_index],
      &folder->envelopes[envelope_index + 1],
      sizeof(ck_envelope_t) * (folder->envelope_count - 1 - envelope_index));

  folder->envelope_count--;
  return 0;
}

static ck_err_t expel_envelope(const ck_envelope_t *envelope) {
  // Expelling only works if the envelope is also disabled.
  if (envelope->enable) {
    return CK_ERR_INVALID_PARAMETER;
  }

  // Loop through folders and remove the envelope if found.
  for (uint8_t i = 0; i < mayor.user_data.folder_count; i++) {
    if (remove_envelope_from_folder(&mayor.user_data.folders[i], envelope) ==
        0) {
      break;
    }
  }
  return CK_OK;
}

static ck_err_t transfer_envelope(uint8_t folder_no,
                                  const ck_envelope_t *envelope) {
  for (uint8_t i = 0; i < mayor.user_data.folder_count; i++) {
    if (remove_envelope_from_folder(&mayor.user_data.folders[i], envelope) ==
        0) {
      break;
    }
  }

  return assign_envelope(folder_no, envelope);
}

static ck_list_t *find_list(ck_list_type_t type, ck_direction_t direction,
                            uint8_t list_no) {
  for (uint8_t i = 0; i < mayor.user_data.list_count; i++) {
    if (mayor.user_data.lists[i].type == type &&
        mayor.user_data.lists[i].direction == direction &&
        mayor.user_data.lists[i].list_no == list_no) {
      return &mayor.user_data.lists[i];
    }
  }
  return NULL;
}

static ck_err_t get_source_list_type(ck_list_type_t target_list_type,
                                     ck_list_type_t *source_list_type) {
  switch (target_list_type) {
    case CK_LIST_LINE:
      *source_list_type = CK_LIST_BIT;
      break;

    case CK_LIST_PAGE:
      *source_list_type = CK_LIST_LINE;
      break;

    case CK_LIST_DOCUMENT:
      *source_list_type = CK_LIST_PAGE;
      break;

    default:
      return CK_ERR_INVALID_PARAMETER;
  }
  return CK_OK;
}
