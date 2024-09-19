/*******************************************************************************
 * @file king.h
 *
 * This file defines the structures and functions needed to implement a capital
 * city. Provides the mandatory king's pages 0-2 and the optional king's pages
 * 16 and 17.
 ******************************************************************************/

#ifndef CK_KING_H
#define CK_KING_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ck-types.h"

/******************************************************************************
 * Struct defining parameters for king's page 0.
 *****************************************************************************/
typedef struct {
  /// City or group address.
  uint8_t address;
  /// Action mode.
  ck_action_mode_t action_mode;
  /// Communication mode.
  ck_comm_mode_t comm_mode;
  /// Communication mode flags to set.
  ck_comm_flags_t comm_flags;
  /// City mode. Only #CK_CITY_MODE_KEEP_CURRENT is predefined. Modes are
  /// defined by mayor and specified in their city's documentation.
  ck_city_mode_t city_mode;
} ck_kp0_args_t;

/******************************************************************************
 * Struct defining parameters for king's page 1.
 *****************************************************************************/
typedef struct {
  /// City or group address.
  uint8_t address;
  /// The mayor's response page in the mayor's document. Defined by the mayor
  /// and should be specified in the documentation. If no response is required,
  /// use CK_KP1_NO_RESPONSE_REQUESTED.
  uint8_t mayor_response_no;
  /// Base number of kingdom. This should be defined by the king.
  uint32_t base_no;
  /// Whether to use standard or extended CAN IDs.
  bool has_extended_id;
} ck_kp1_args_t;

/******************************************************************************
 * Struct defining parameters for king's page 2. Compressed envelopes are not
 * implemented.
 *****************************************************************************/
typedef struct {
  /// City or group address.
  uint8_t address;
  /// Envelope argument.
  ck_envelope_t envelope;
  /// Folder number.
  uint8_t folder_no;
  /// Envelope action to take.
  ck_envelope_action_t envelope_action;
} ck_kp2_args_t;

/******************************************************************************
 * Struct defining parameters for king's page 16.
 *****************************************************************************/
typedef struct {
  /// City or group address.
  uint8_t address;
  /// Folder number.
  uint8_t folder_no;
  /// at most CK_CAN_MAX_DLC.
  uint8_t dlc;
  /// The rtr bit.
  bool has_rtr;
  /// Direction of the target document.
  ck_direction_t direction;
  /// Document action to take.
  ck_document_action_t document_action;
  /// Enable use of the folder.
  bool enable_folder;
  /// Document list number.
  uint8_t list_no;
  /// Document number in list.
  uint8_t document_no;
} ck_kp16_args_t;

/******************************************************************************
 * Struct defining parameters for king's page 17.
 * We use the term "record" to denote an item in a list.
 *****************************************************************************/
typedef struct {
  /// City or group address.
  uint8_t address;
  /// Direction of the source and target records.
  ck_direction_t direction;
  /// Target list type
  ck_list_type_t list_type;
  /// Source list number.
  uint8_t source_list_no;
  /// Source record number.
  uint8_t source_record_no;
  /// Target list number.
  uint8_t target_list_no;
  /// Target record number.
  uint8_t target_record_no;
  /// Where to put the specified record in the target, i.e.
  /// a form number in a document, a line number on a page,
  /// or a bit number on a line.
  uint8_t position;
} ck_kp17_args_t;

/*******************************************************************************
 * KP0 terminates the setup phase.
 *
 * Orders a mayor to set his city into a specific working mode, e.g., in a "Run"
 *or "Freeze" mode.
 *
 * @param args ck_kp0_args_t value with the desired values.
 * @param page will be populated with the created king's page.
 *
 * @return #CK_ERR_INVALID_ACTION_MODE if the given action mode is invalid.
 * @return #CK_ERR_INVALID_COMM_MODE if the given comm mode is invalid.
 * @return #CK_OK on success.
 ******************************************************************************/
ck_err_t ck_create_kings_page_0(const ck_kp0_args_t *args, ck_page_t *page);

/*******************************************************************************
 * KP1 is the initiating page.
 *
 * It provides the base number and asks for mayor’s response.
 *
 * @param args ck_kp1_args_t value with the desired values.
 * @param page will be populated with the created king's page.
 *
 * @return #CK_ERR_INVALID_CAN_ID if the given CAN ID is out of bounds.
 * @return #CK_OK on success.
 ******************************************************************************/
ck_err_t ck_create_kings_page_1(const ck_kp1_args_t *args, ck_page_t *page);

/*******************************************************************************
 * KP2 is used to assign, expel or transfer envelopes from/to folders.
 *
 * @param args ck_kp2_args_t value with the desired values.
 * @param page will be populated with the created king's page.
 *
 * @return #CK_ERR_INCOMPATIBLE_PARAMS if some of the args are incompatible.
 * @return #CK_ERR_INVALID_CAN_ID if the given CAN ID is out of bounds.
 * @return #CK_OK on success.
 *******************************************************************************/
ck_err_t ck_create_kings_page_2(const ck_kp2_args_t *args, ck_page_t *page);

/*******************************************************************************
 * KP8 is used to set the CAN bit timing settings for a city.
 *
 * @param address city or group address.
 * @param bit_timing ck_can_bit_timing_t with the desired values.
 * @param page will be populated with the created king's page.
 *
 * @return #CK_ERR_INCOMPATIBLE_PARAMS if some of the args are incompatible.
 * @return #CK_ERR_INVALID_CAN_ID if the given CAN ID is out of bounds.
 * @return #CK_OK on success.
 *******************************************************************************/
ck_err_t ck_create_kings_page_8(uint8_t address,
                                const ck_can_bit_timing_t *bit_timing,
                                ck_page_t *page);

/*******************************************************************************
 * KP16 Sets the folder label and/or places a document into a folder.
 *
 * @param args ck_kp16_args_t value with the desired values.
 * @param page will be populated with the created king's page.
 *
 * @return #CK_ERR_INVALID_FOLDER_NUMBER if a reserved folder number is
 *given.
 * @return #CK_ERR_INVALID_CAN_DLC if the given DLC value is out of bounds.
 * @return #CK_OK on success.
 ******************************************************************************/
ck_err_t ck_create_kings_page_16(const ck_kp16_args_t *args, ck_page_t *page);

/*******************************************************************************
 * KP17 Creates a document, a page or a line from predefined page, line, or bit
 * lists.
 *
 * @param args ck_kp17_args_t value with the desired values.
 * @param page will be populated with the created king's page.
 *
 * @return #CK_ERR_INVALID_LIST_TYPE if the given list type is invalid.
 * @return #CK_OK on success.
 *******************************************************************************/
ck_err_t ck_create_kings_page_17(const ck_kp17_args_t *args, ck_page_t *page);

#ifdef __cplusplus
}
#endif

#endif /* CK_KING_H */
