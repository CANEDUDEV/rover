/*
 * This file defines the structures and functions needed to implement a capital
 * city. Provides the mandatory king's pages 0-2 and the optional king's pages
 * 16 and 17.
 */

#ifndef CK_KING_H
#define CK_KING_H

#include "postmaster.h"
#include "types.h"

/*
 * Struct defining parameters for king's page 0.
 */
typedef struct {
  uint8_t address;               // City or group address.
  ck_action_mode_t action_mode;  // Action mode.
  ck_comm_mode_t comm_mode;      // Communication mode.
  ck_city_mode_t city_mode;      // City mode. Only CK_CITY_MODE_KEEP_CURRENT is
                                 // predefined. Modes are defined by mayor and
                                 // specified in their city's documentation.
} ck_kp0_args_t;

/*
 * Struct defining parameters for in king's page 1.
 */
typedef struct {
  uint8_t address;            // City or group address.
  uint8_t mayor_response_no;  // the mayor's response page in the mayor's
                              // document. Defined by the mayor and should be
                              // specified in the documentation. If no response
                              // is required, use CK_KP1_NO_RESPONSE_REQUESTED.
  uint32_t
      base_no;  // Base number of kingdom. This should be defined by the king.
  bool has_extended_id;  // Whether to use standard or extended CAN IDs.
} ck_kp1_args_t;

/*
 * Struct defining parameters for king's page 2.
 * Compressed envelopes are not implemented.
 */
typedef struct {
  uint8_t address;         // City or group address.
  ck_envelope_t envelope;  // Envelope argument.
  uint8_t folder_no;       // Folder number.
  ck_envelope_action_t envelope_action;
} ck_kp2_args_t;

/*
 * Struct defining parameters for king's page 16.
 */
typedef struct {
  uint8_t address;    // City or group address.
  uint8_t folder_no;  // Folder number.
  uint8_t dlc;        // at most CAN_MAX_DLC.
  bool has_rtr;       // The rtr bit.
  ck_direction_t direction;
  ck_document_action_t document_action;
  bool enable_folder;   // Enable use of the folder.
  uint8_t list_no;      // Document list number.
  uint8_t document_no;  // Document number in list.
} ck_kp16_args_t;

/*
 * Struct defining parameters for king's page 17.
 * We use the term "record" to denote an item in a list.
 */
typedef struct {
  uint8_t address;  // City or group address.
  ck_direction_t direction;
  ck_list_type_t list_type;  // Target list type
  uint8_t source_list_no;    // Source list number.
  uint8_t source_record_no;  // Source record number.
  uint8_t target_list_no;    // Target list number.
  uint8_t target_record_no;  // Target record number.
  uint8_t position;  // Where to put the specified record in the target, i.e. a
                     // form number in a document, a line number on a page, or a
                     // bit number on a line.
} ck_kp17_args_t;

/*
 * KP0 terminates the setup phase. Orders a mayor to set his city into a
 * specific working mode, e.g., in a "Run" or "Freeze" mode.
 *
 * Return codes
 *      - CK_OK
 */
ck_err_t ck_create_kings_page_0(const ck_kp0_args_t *args, ck_page_t *page);

/*
 * KP1 is the initiating page. It provides the base number and asks for
 * mayor’s response.
 *
 * Return codes
 *      - CK_OK
 *      - CK_ERR_INVALID_CAN_ID
 */
ck_err_t ck_create_kings_page_1(const ck_kp1_args_t *args, ck_page_t *page);

/*
 * KP2 is used to assign envelopes to or expel envelopes from a folder. It can
 * also be used to move an envelope from one folder to another.
 *
 * Return codes
 *      - CK_OK
 *      - CK_ERR_INVALID_CAN_ID
 *      - CK_ERR_KP_INCOMPATIBLE_PARAMS
 */
ck_err_t ck_create_kings_page_2(const ck_kp2_args_t *args, ck_page_t *page);

/*
 * KP16 Sets the folder label and/or places a document into a folder.
 *
 * Return codes
 *      - CK_OK
 *      - CK_ERR_KP_INVALID_FOLDER_NUMBER
 *      - CK_ERR_KP_DOCUMENT_INSERT_WITHOUT_REMOVAL
 *      - CK_ERR_INVALID_CAN_DLC
 */
ck_err_t ck_create_kings_page_16(const ck_kp16_args_t *args, ck_page_t *page);

/*
 * KP17 Creates a document, a page form or a line form from
 * predefined page, line, or bit form lists.
 *
 * Return codes
 *      - CK_OK.
 */
ck_err_t ck_create_kings_page_17(const ck_kp17_args_t *args, ck_page_t *page);

#endif /* CK_KING_H */
