/*******************************************************************************
 * @file types.h
 *
 * CAN Kingdom building blocks.
 *
 * - Bit: a bit.
 * - Line: A line is 8 bits, a byte.
 * - Envelope: a CAN ID (or rather, the arbitration field)
 * - Page: 0-8 lines (a CAN data field, i.e. maximum 8 bytes).
 * - Letter: A page in an envelope (a CAN frame).
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
 ******************************************************************************/

#ifndef CK_TYPES_H
#define CK_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/// The max value for the CAN DLC. Corresponds to 8 bytes.
#define CK_CAN_MAX_DLC 8
/// The max value for a CAN ID when using standard (11-bit) identifiers.
#define CK_CAN_MAX_STD_ID ((1UL << 11) - 1)
/// The max value for a CAN ID when using extended (29-bit) identifiers.
#define CK_CAN_MAX_EXT_ID ((1UL << 29) - 1)
/// For extracting IDs out of King's pages.
#define CK_CAN_ID_MASK 0x1FFFFFFF

/// The highest possible 40-bit EAN number.
#define CK_MAX_EAN_NO ((1ULL << 40) - 1)

/// Default number of the king's folder.
#define CK_KINGS_FOLDER_NO 0

/// Default number of the mayor's folder.
#define CK_MAYORS_FOLDER_NO 1

/// The default letter's CAN ID.
#define CK_DEFAULT_LETTER_ENVELOPE 2031

/// For use with king's page 1 when requesting a mayor's response.
#define CK_NO_RESPONSE_REQUESTED 0xFF

/// Max number of lines per page.
#define CK_MAX_LINES_PER_PAGE CK_CAN_MAX_DLC

/* Limit some structures' maximum sizes by default. This makes it easier to
 * implement the library and reduces the risk for segfaults on the user end.
 * The limits can be overriden during compilation.
 */
#ifndef CK_MAX_ENVELOPES_PER_FOLDER
/// Max number of envelopes that can be assigned to a folder.
#define CK_MAX_ENVELOPES_PER_FOLDER 8
#endif
#ifndef CK_MAX_PAGES_PER_DOCUMENT
/// Max number of pages in a document.
#define CK_MAX_PAGES_PER_DOCUMENT 8
#endif
#ifndef CK_MAX_RECORDS_PER_LIST
/// Max number of records in a list.
#define CK_MAX_RECORDS_PER_LIST 8
#endif

/// Error codes.
typedef enum {
  /// OK.
  CK_OK = 0,
  /// Invalid CAN ID.
  CK_ERR_INVALID_CAN_ID,
  /// Invalid CAN DLC.
  CK_ERR_INVALID_CAN_DLC,
  /// Invalid ck_can_bit_timing_t.
  CK_ERR_INVALID_CAN_BIT_TIMING,
  /// Invalid King's letter.
  CK_ERR_INVALID_KINGS_LETTER,
  /// Unsupported King's page.
  CK_ERR_UNSUPPORTED_KINGS_PAGE,
  /// Some parameters are not compatible.
  CK_ERR_INCOMPATIBLE_PARAMS,
  /// User tried to set a reserved folder number.
  CK_ERR_INVALID_FOLDER_NUMBER,
  /// Invalid #ck_action_mode_t
  CK_ERR_INVALID_ACTION_MODE,
  /// Invalid #ck_comm_mode_t.
  CK_ERR_INVALID_COMM_MODE,
  /// Invalid #ck_list_type_t.
  CK_ERR_INVALID_LIST_TYPE,
  /// Missing input argument to function.
  CK_ERR_MISSING_PARAMETER,
  /// List, folder, document or page is full.
  CK_ERR_CAPACITY_REACHED,
  /// Item not found in list or folder.
  CK_ERR_ITEM_NOT_FOUND,
  /// Library has not been initialized yet.
  CK_ERR_NOT_INITIALIZED,
  /// Failed to send a letter.
  CK_ERR_SEND_FAILED,
  /// Returned when setting a mode fails.
  CK_ERR_SET_MODE_FAILED,
  /// Error code from boolean return type functions
  CK_ERR_FALSE,
  /// Failed to access some peripheral.
  CK_ERR_PERIPHERAL,
  /// Input argument to function is wrong in some way.
  CK_ERR_INVALID_PARAMETER,
} ck_err_t;

/// Supported king's pages.
typedef enum {
  /// King's page 0.
  CK_KP0 = 0,
  /// King's page 1.
  CK_KP1 = 1,
  /// King's page 2.
  CK_KP2 = 2,
  /// King's page 16.
  CK_KP16 = 16,
  /// King's page 17.
  CK_KP17 = 17,
} ck_kings_page_t;

/// Action mode as defined by the CAN Kingdom specification.
typedef enum {
  /// Keep the current mode.
  CK_ACTION_MODE_KEEP_CURRENT = 0x0,
  /// Start the city.
  CK_ACTION_MODE_RUN = 0x1,
  /// Stop the city.
  CK_ACTION_MODE_FREEZE = 0x2,
  /// Reset the city.
  CK_ACTION_MODE_RESET = 0x3,
} ck_action_mode_t;

/// Communication mode as defined by the CAN Kingdom specification.
typedef enum {
  /// Keep the current mode.
  CK_COMM_MODE_KEEP_CURRENT = 0x0,
  /// Set the CAN controller in silent mode.
  CK_COMM_MODE_SILENT = 0x1,
  /// Set the CAN controller in listen-only mode.
  CK_COMM_MODE_LISTEN_ONLY = 0x2,
  /// Enable CAN communication.
  CK_COMM_MODE_COMMUNICATE = 0x3,
} ck_comm_mode_t;

/// Communication mode flags, used in KP0.
typedef enum {
  /// Reset communication by going through the startup sequence with new bit
  /// timing settings.
  CK_COMM_RESET = 0x4,
  /// Skip listening for a good message during startup sequence.
  CK_COMM_SKIP_LISTEN = 0x8,
  /// Don't skip listening for a good message during startup sequence.
  CK_COMM_DONT_SKIP_LISTEN = 0x10,
  /// Skip 200ms wait for default letter during startup sequence.
  CK_COMM_SKIP_WAIT = 0x20,
  /// Don't 200ms wait for default letter during startup sequence.
  CK_COMM_DONT_SKIP_WAIT = 0x40,
} ck_comm_flags_t;

/// Only #CK_CITY_MODE_KEEP_CURRENT is defined, the rest is defined by the user.
typedef enum {
  /// Keep the current mode.
  CK_CITY_MODE_KEEP_CURRENT = 0,
} ck_city_mode_t;

/// Direction enum.
typedef enum {
  /// Receive direction.
  CK_DIRECTION_RECEIVE = 0,
  /// Transmit direction.
  CK_DIRECTION_TRANSMIT = 1,
} ck_direction_t;

/// Page structure, represents the data in a CAN frame.
typedef struct {
  /// Number of lines on page.
  uint8_t line_count;
  /// Line data.
  uint8_t lines[CK_MAX_LINES_PER_PAGE];
} ck_page_t;

/// Envelope structure, represents the header of a CAN frame.
typedef struct {
  /// CAN ID
  uint32_t envelope_no;
  /// Whether envelope is enabled or not.
  bool enable;
  /// Whether envelope uses standard or extended CAN IDs.
  bool has_extended_id;
  /// Remote envelope (RTR bit).
  bool is_remote;
  /// Compressed envelope. Not supported at the moment.
  bool is_compressed;
} ck_envelope_t;

/// Letter structure, for creating CAN messages from folders.
typedef struct {
  ck_envelope_t envelope;
  ck_page_t page;
} ck_letter_t;

/*******************************************************************************
 * The document structure contains a list of pages that should be sent using the
 * same envelope. If a document contains more than one page, the pages should
 * have a pagination number in order to detect errors during transmission, such
 * as omission of pages or duplicates transmission.
 ******************************************************************************/
typedef struct {
  /// Direction of the document.
  ck_direction_t direction;
  /// How many pages the document contains.
  uint8_t page_count;
  /// List of pointers to pages.
  ck_page_t *pages[CK_MAX_PAGES_PER_DOCUMENT];
} ck_document_t;

/*******************************************************************************
 * The folder structure links a document to a set of envelopes. It contains
 * information that can be set by both the king and the mayor. Describes how to
 * send or receive a document. In practice, it is used to assign CAN IDs to
 * various messages and control whether or not they should be sent.
 ******************************************************************************/
typedef struct {
  /// Folder number.
  uint8_t folder_no;
  /// Document list number.
  uint8_t doc_list_no;
  /// Document number.
  uint8_t doc_no;
  /// Direction of the folder.
  ck_direction_t direction;
  /// CAN DLC
  uint8_t dlc;
  /// CAN RTR bit
  bool has_rtr;
  /// Folder enabled / disabled.
  bool enable;
  /// How many envelopes have been assigned.
  uint8_t envelope_count;
  /// Assigned envelopes. The specification allows more than one envelope to be
  /// assigned to a folder.
  ck_envelope_t envelopes[CK_MAX_ENVELOPES_PER_FOLDER];
} ck_folder_t;

/*******************************************************************************
 * Some list types for storing predefined data in a city, which can be used by
 * the king to construct new pages or documents.
 ******************************************************************************/
typedef enum {
  /// Bit list.
  CK_LIST_BIT = 0,
  /// Line list.
  CK_LIST_LINE = 1,
  /// Page list.
  CK_LIST_PAGE = 2,
  /// Document list.
  CK_LIST_DOCUMENT = 3,
} ck_list_type_t;

/*******************************************************************************
 * A generic structure for storing line lists, page flists or document lists.
 *
 * The type of the records parameter depends on the value of the type parameter.
 *
 * | list_type value   | record value    |
 * |-------------------|-----------------|
 * | #CK_LIST_BIT      | uint8_t *       |
 * | #CK_LIST_LINE     | uint8_t *       |
 * | #CK_LIST_PAGE     | ck_page_t *     |
 * | #CK_LIST_DOCUMENT | ck_document_t * |
 *
 * The `uint8_t *` in a #CK_BIT_LIST will be treated as a contiguous bit array.
 * This means the record count should reflect the number of bits, not the number
 * of bytes in the list.
 *
 ******************************************************************************/
typedef struct {
  /// The type of items this list contains.
  ck_list_type_t type;
  /// #CK_DIRECTION_RECEIVE or #CK_DIRECTION_TRANSMIT.
  ck_direction_t direction;
  /// List number. Must be unique for each direction and type.
  uint8_t list_no;
  /// Number of records in the list.
  uint8_t record_count;
  /// List of pointers to records.
  void *records[CK_MAX_RECORDS_PER_LIST];
} ck_list_t;

/*******************************************************************************
 * Defines envelope actions for use with KP2.
 ******************************************************************************/
typedef enum {
  CK_ENVELOPE_NO_ACTION = 0,
  /// Assign envelope to folder.
  CK_ENVELOPE_ASSIGN = 1,
  /// Expel envelope from previous assignment. Envelope must be disabled in the
  /// same message for this to work.
  CK_ENVELOPE_EXPEL = 2,
  /// Move envelope to new folder.
  CK_ENVELOPE_TRANSFER = 3,
} ck_envelope_action_t;

/*******************************************************************************
 * Defines document actions for use with KP16.
 ******************************************************************************/
typedef enum {
  CK_DOCUMENT_NO_ACTION = 0,
  /// Remove document from folder.
  CK_DOCUMENT_REMOVE = 2,
  /// Insert document into folder. A folder contains at most one document.
  /// Inserting a document into a folder with a document already in it will
  /// replace the old document with the new one.
  CK_DOCUMENT_INSERT = 3,
} ck_document_action_t;

/*******************************************************************************
 * Defines the CAN bit timing parameters needed to set the postoffice settings.
 *
 * The value of Phase segment 1 + propagation segment will be inferred from the
 * following equation: `time_quanta = 1 + prop_seg + phase_seg1 + phase_seg2`,
 * where `8 <= time_quanta <= 25` and `2 <= phase_seg2 <= 8`.
 ******************************************************************************/
typedef struct {
  /// Defines the clock prescaler, which will determine the bit rate. It's
  /// important to know the clock frequency of the CAN peripheral to set the
  /// correct bit rate.
  ///
  /// `1 <= prescaler <= 32`
  uint8_t prescaler;

  uint8_t time_quanta;  /// Number of time quanta.

  uint8_t phase_seg2;  /// Phase segment 2.

  /// Synchronization jump width, `1 <= sjw <= 4`. Must also be less than the
  /// minimum of phase_seg1 and phase_seg2.
  uint8_t sjw;

} ck_can_bit_timing_t;

/*******************************************************************************
 * Check if the specified action mode is a valid action mode.
 *
 * @param mode #ck_action_mode_t to check.
 *
 * @return #CK_ERR_INVALID_ACTION_MODE if mode is invalid.
 * @return #CK_OK on success.
 ******************************************************************************/
ck_err_t ck_check_action_mode(ck_action_mode_t mode);

/*******************************************************************************
 * Check if the specified communication mode is a valid communication mode.
 *
 * @param mode #ck_comm_mode_t to check.
 *
 * @return #CK_ERR_INVALID_COMM_MODE if mode is invalid.
 * @return #CK_OK on success.
 ******************************************************************************/
ck_err_t ck_check_comm_mode(ck_comm_mode_t mode);

/*******************************************************************************
 * Check if the specified list type is valid.
 *
 * @param type #ck_list_type_t to check.
 *
 * @return #CK_ERR_INVALID_LIST_TYPE if type is invalid.
 * @return #CK_OK on success.
 ******************************************************************************/
ck_err_t ck_check_list_type(ck_list_type_t type);

/*******************************************************************************
 * Check if the given ck_can_bit_timing_t is valid.
 *
 * The checking performed by the function relate to the CAN standard for what is
 * allowed and what is not allowed. Some hardware may be able to set some values
 * that this function will reject. In that case, the user will have to perform
 * validation themselves.
 *
 * @param bit_timing pointer to parameters to check.
 *
 * @return #CK_ERR_INVALID_CAN_BIT_TIMING if bit_timing is invalid.
 * @return #CK_OK on success.
 ******************************************************************************/
ck_err_t ck_check_can_bit_timing(const ck_can_bit_timing_t *bit_timing);

/*******************************************************************************
 * Returns a CAN Kingdom default letter.
 *
 * The default letter is a letter with CAN ID 2031 and 8 lines each containing
 * `0xAA`.
 *
 * @return the created letter.
 ******************************************************************************/
ck_letter_t ck_default_letter(void);

/*******************************************************************************
 * Returns bit timing parameters specifying a bit rate of 125 Kbit/s and a
 * sampling point of 87.5%.
 *
 * @return the ck_can_bit_timing_t.
 ******************************************************************************/
ck_can_bit_timing_t ck_default_bit_timing(void);

#ifdef __cplusplus
}
#endif

#endif /* CK_TYPES_H */
