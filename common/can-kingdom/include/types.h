/*
 * CAN Kingdom building blocks
 *
 * Bit: a bit.
 * Line: A line is 8 bits, a byte.
 * Envelope: a CAN ID (or rather, the arbitration field)
 * Page: 0-8 lines (a CAN data field, i.e. maximum 8 bytes).
 * Letter: A page in an envelope (a CAN frame).
 *
 * Form: Explains the contents of a a page, line by line. It also specified
 * whether the city produces this information or requires it. Forms are numbered
 * using a list number and a form number. One list can contain up to 255 forms,
 * and one city can have up to 255 lists. The forms should be specified in a
 * city's documentation.
 *
 * The mayor can use the form to encode and decode data. For two cities to
 * exchange data, their forms must match exactly. Thus, the mayor can choose to
 * either specify multiple forms with the same information to fit different
 * systems, or he can give the king the possibility to compose forms from
 * predefined line lists.
 *
 * Forms do not need to be implemented in code, only in documentation.
 *
 * Document: a set of pages that should be carried in the same envelope. The
 * documents should be specified in a city's documentation. A document may be
 * composed of one or multiple pages. For such documents, every page should have
 * one line (or at least some bits) that specify the page number, since all
 * the data is sent in one envelope. The envelope number is not specified in a
 * document, but in a folder (see below).
 *
 * New documents can be constructed by the king if the mayor provides
 * predefined bit, line or page lists. The mayor must also allocate space for
 * any new documents, either beforehand or dynamically.
 *
 * Lists: Provided by mayors to list the documents, forms, lines and bits the
 * city provides or requires. The king can use these lists to design their own
 * documents according to the system's needs, or use the primitives provided
 * as-is. Lists are identified using direction (Transmit or Receive) and number,
 * i.e. T0, T1, etc. or R0, R1, etc.
 *
 * Folder: structure for how a document should be used in a kingdom. A city may
 * provide many documents, however the kingdom founder (system designer) might
 * only need some of them. Then for every document the founder wants to use,
 * they create a folder to hold it. The folders are what makes it possible to go
 * from document to a set of CAN frames.
 *
 * Each folder is labeled. The label contains the folder number, document list
 * number, the document number, whether to transmit or receive said document,
 * the CAN control field, the RTR bit of the CAN message, whether to enable the
 * folder or not, the envelopes assigned to it, and which of these envelopes
 * should be enabled or disabled. A folder may be allocated more than one
 * envelope. A folder may be empty or contain at most one document.
 *
 * Folders 0 and 1 are reserved by CAN Kingdom for the King's document and the
 * Mayor's document, respectively.
 *
 * Record: a record is a collective name for an item in a list, i.e. a document
 * in a document list is called a record and a form in a form list is a record,
 * etc. A record is identified using the same notation as lists, with the added
 * record number. For instance, record number 1 in list T0 is denoted T0.1.
 *
 * It should be noted that not all parts of the building blocks should be
 * implemented in code. Some of the information should reside in the
 * documentation of the cities. In the document for example, it is important
 * that the document designer provides the list and document number, in order
 * for the system designer to be able to differentiate between various documents
 * in code, however the description for how to interpret the pages in the
 * document should be provided in the documentation.
 */

#ifndef CK_TYPES_H
#define CK_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

// Some helpers
#define CK_CAN_MAX_DLC 8
#define CK_CAN_MAX_STD_ID ((1UL << 11) - 1)  // 11-bit IDs
#define CK_CAN_MAX_EXT_ID ((1UL << 29) - 1)  // 29-bit IDs
#define CK_CAN_ID_MASK 0x1FFFFFFF  // For extracting IDs out of King's pages

#define CK_MAX_EAN_NO ((1ULL << 40) - 1)  // 40-bit EAN

// Default folders
#define CK_KINGS_FOLDER_NO 0
#define CK_MAYORS_FOLDER_NO 1

#define CK_DEFAULT_KP_COUNT 5  // Number of king's pages in a standard system

#define CK_DEFAULT_LETTER_ENVELOPE 2031  // For constructing the default letter

#define CK_NO_RESPONSE_REQUESTED 0xFF  // For use with king's page 1

#define CK_MAX_LINES_PER_PAGE CK_CAN_MAX_DLC

/* Limit some structures' maximum sizes by default. This makes it easier to
 * implement the library and reduces the risk for segfaults on the user end.
 * The limits can be overriden during compilation.
 */
#ifndef CK_MAX_ENVELOPES_PER_FOLDER
#define CK_MAX_ENVELOPES_PER_FOLDER 8
#endif
#ifndef CK_MAX_PAGES_PER_DOCUMENT
#define CK_MAX_PAGES_PER_DOCUMENT 8
#endif
#ifndef CK_MAX_RECORDS_PER_LIST
#define CK_MAX_RECORDS_PER_LIST 8
#endif

// Error codes
typedef enum {
  CK_OK = 0,
  CK_ERR_INVALID_CAN_ID,
  CK_ERR_INVALID_CAN_DLC,
  CK_ERR_INVALID_KINGS_LETTER,
  CK_ERR_UNSUPPORTED_KINGS_PAGE,
  CK_ERR_INCOMPATIBLE_PARAMS,
  CK_ERR_INVALID_FOLDER_NUMBER,
  CK_ERR_INVALID_ACTION_MODE,
  CK_ERR_INVALID_COMM_MODE,
  CK_ERR_INVALID_LIST_TYPE,
  CK_ERR_INVALID_PARAMETER,
  CK_ERR_MISSING_PARAMETER,
  CK_ERR_CAPACITY_REACHED,
  CK_ERR_ITEM_NOT_FOUND,
  CK_ERR_NOT_INITIALIZED,
} ck_err_t;

// Supported king's pages
typedef enum {
  CK_KP0 = 0,
  CK_KP1 = 1,
  CK_KP2 = 2,
  CK_KP16 = 16,
  CK_KP17 = 17,
} ck_kings_page_t;

typedef enum {
  CK_ACTION_MODE_KEEP_CURRENT = 0x00,
  CK_ACTION_MODE_RUN = 0x01,
  CK_ACTION_MODE_FREEZE = 0x10,
  CK_ACTION_MODE_RESET = 0x11,
} ck_action_mode_t;

typedef enum {
  CK_COMM_MODE_KEEP_CURRENT = 0x00,
  CK_COMM_MODE_SILENT = 0x01,
  CK_COMM_MODE_LISTEN_ONLY = 0x10,
  CK_COMM_MODE_COMMUNICATE = 0x11,
} ck_comm_mode_t;

// Only keep current mode is defined, the rest is defined by the user.
typedef enum {
  CK_CITY_MODE_KEEP_CURRENT = 0,
} ck_city_mode_t;

typedef enum {
  CK_DIRECTION_RECEIVE = 0,
  CK_DIRECTION_TRANSMIT = 1,
} ck_direction_t;

typedef struct {
  uint8_t line_count;  // Number of lines on page
  uint8_t lines[CK_MAX_LINES_PER_PAGE];
} ck_page_t;

typedef struct {
  uint32_t envelope_no;  // CAN ID
  bool enable;           // Whether envelope is enabled or not.
  bool has_extended_id;  // Whether envelope uses standard or extended CAN IDs.
  bool is_remote;        // Remote envelope (RTR bit).
  bool is_compressed;    // Compressed envelope. Not supported at the moment.
} ck_envelope_t;

typedef struct {
  ck_envelope_t envelope;
  ck_page_t page;
} ck_letter_t;

/*
 * The document structure contains a list of pages that should be sent using the
 * same envelope. If a document contains more than one page, the pages should
 * have a pagination number in order to detect errors during transmission, such
 * as omission of pages or duplicates transmission.
 */
typedef struct {
  ck_direction_t direction;
  uint8_t page_count;  // How many pages the document contains.
  ck_page_t *pages[CK_MAX_PAGES_PER_DOCUMENT];  // List of pointers to pages.
} ck_document_t;

/*
 * The folder structure links a document to a set of envelopes. It contains
 * information that can be set by both the king and the mayor. Describes how to
 * send or receive a document. In practice, it is used to assign CAN IDs to
 * various messages and control whether or not they should be sent.
 */
typedef struct {
  uint8_t folder_no;
  uint8_t doc_list_no;  // Document list number.
  uint8_t doc_no;       // Document number.
  ck_direction_t direction;
  uint8_t dlc;             // CAN DLC
  bool has_rtr;            // CAN RTR bit
  bool enable;             // Folder enabled / disabled.
  uint8_t envelope_count;  // How many envelopes have been assigned.
  ck_envelope_t
      envelopes[CK_MAX_ENVELOPES_PER_FOLDER];  // Assigned envelopes. The
                                               // specification allows more than
                                               // one envelope to be assigned to
                                               // a folder.
} ck_folder_t;

/*
 * Some list types for storing predefined data in a city, which can be used by
 * the king to construct new pages or documents.
 */
typedef enum {
  CK_LIST_BIT = 0,
  CK_LIST_LINE = 1,
  CK_LIST_PAGE = 2,
  CK_LIST_DOCUMENT = 3,
} ck_list_type_t;

/*
 * A generic struct for storing line form lists, page form lists and document
 * lists.
 *
 * The type of the `record` parameter depends on the value of the `list_type`
 * parameter.
 *
 * | `list_type` value | `record` type   |
 * |-------------------|-----------------|
 * | CK_BIT_LIST       | uint8_t *       |
 * | CK_LINE_LIST      | uint8_t *       |
 * | CK_PAGE_LIST      | ck_page_t *     |
 * | CK_DOCUMENT_LIST  | ck_document_t * |
 *
 * The uint8_t * in a CK_BIT_LIST will be treated as a contiguous bit array.
 * This means the record count should reflect the number of bits, not the number
 * of bytes in the list.
 *
 */
typedef struct {
  ck_list_type_t type;       // The type of items this list contains.
  ck_direction_t direction;  // CK_DIRECTION_RECEIVE or CK_DIRECTION_TRANSMIT.
  uint8_t list_no;   // List number. Must be unique for each direction and type.
  uint8_t capacity;  // Number of records this list can hold.
  void *records[CK_MAX_RECORDS_PER_LIST];  // List of pointers to records.
} ck_list_t;

/*
 * Defines envelope actions for use with KP2.
 */
typedef enum {
  CK_ENVELOPE_NO_ACTION = 0,
  CK_ENVELOPE_ASSIGN = 1,  // Assign envelope to folder.
  CK_ENVELOPE_EXPEL =
      2,  // Expel envelope from previous assignment. Envelope must
          // be disabled in the same message for this to work.
  CK_ENVELOPE_TRANSFER = 3,  // Move envelope to new folder.
} ck_envelope_action_t;

/*
 * Defines document actions for use with KP16.
 */
typedef enum {
  CK_DOCUMENT_NO_ACTION = 0,
  CK_DOCUMENT_REMOVE = 2,  // Remove document from folder.
  CK_DOCUMENT_INSERT =
      3,  // Insert document into folder. A folder contains at most one
          // document. Inserting a document into a folder with a document
          // already in it will replace the old document with the new one.

} ck_document_action_t;

/*
 * Check if the specified action mode is a valid action mode.
 */
ck_err_t ck_check_action_mode(ck_action_mode_t mode);

/*
 * Check if the specified communication mode is a valid communication mode.
 */
ck_err_t ck_check_comm_mode(ck_comm_mode_t mode);

/*
 * Check if the specified list type is valid.
 */
ck_err_t ck_check_list_type(ck_list_type_t type);

/*
 * Returns the CAN Kingdom default letter, i.e. a letter with CAN ID 2031
 * and 8 lines each containing 0xAA.
 */
ck_letter_t default_letter(void);

#ifdef __cplusplus
}
#endif

#endif /* CK_TYPES_H */
