/*******************************************************************************
 * @file mayor.h
 *
 * Functions to set up a mayor in a CAN Kingdom system.
 *
 * The idea is that the user will define some parameters for their mayor as
 * specified by the ck_mayor_t struct, then the library will initialize its
 * state from these parameters when the user call ck_mayor_init. Then,
 * ck_process_kings_letter can be used to act on commands from the king. The
 * postmaster implementation is responsible for parsing CAN messages and
 * converting them to letters, then the user needs to call
 * ck_process_kings_letter to process the letter if it's a king's letter.
 *
 ******************************************************************************/

#ifndef CK_MAYOR_H
#define CK_MAYOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "types.h"

#ifndef CK_MAX_GROUPS_PER_CITY
/// Limited to 6 groups for ease of implementation.
/// Can be redefined during compilation if wanted.
#define CK_MAX_GROUPS_PER_CITY 6
#endif

/*******************************************************************************
 * Struct contains pointers to parameters that should be defined by the user.
 * The parameters are used by the mayor library to initialize the library state.
 ******************************************************************************/
typedef struct {
  // The EAN and serial numbers are used to identify the device. They are each
  // 40 bits, but represented as uint64_t. Bit fields could be used, but then
  // it's impossible to detect if the user has provided an invalid number as it
  // would just overflow.

  /// 40-bit EAN-13 number.
  uint64_t ean_no;

  /// 40-bit serial number.
  uint64_t serial_no;

  /// The city address. Must not be 0. Can be changed by the king.
  uint8_t city_address;

  /// The group addresses. Can be changed by the king.
  /// The city always belongs to group 0 as well.
  uint8_t group_addresses[CK_MAX_GROUPS_PER_CITY];

  /// The base number of the kingdom. Can be set by the mayor if known
  /// beforehand, otherwise the king will set it.
  uint32_t base_no;

  bool has_extended_id;  // Whether to use extended CAN IDs for the base number.

  /// Pointer to function for setting the action mode.
  /// Should return CK_OK on success.
  ck_err_t (*set_action_mode)(ck_action_mode_t);

  /// Pointer to function for setting the communication mode.
  /// Should return CK_OK on success.
  ck_err_t (*set_comm_mode)(ck_comm_mode_t);

  /// Pointer to function for setting the city mode. City modes are defined by
  /// the mayor. Should return CK_OK on success.
  ck_err_t (*set_city_mode)(ck_city_mode_t);

  /// Number of folders the mayor has. Must be at least 2 for the king's and
  /// mayor's folders, which are initialized by ck_mayor_init().
  uint8_t folder_count;

  /// Pointer to user-allocated folders. Folder numbers 0 and 1 are reserved by
  /// CAN Kingdom for the king's document and the mayor's document,
  /// respectively.
  ck_folder_t *folders;

  /// Number of lists this mayor has. Must be at least 2, since we need at least
  /// one transmit document list and one receive document list.
  uint8_t list_count;

  /// Pointer to all the mayor's lists, such as document lists, page lists, and
  /// line lists. The mandatory lists are the transmit document list and the
  /// receive document list, which both need to have capacity 1. Record no 0 in
  /// the transmit document list and the receive document list is reserved by
  /// CAN Kingdom and will be set up by ck_mayor_init().
  ck_list_t *lists;

} ck_mayor_t;

/*******************************************************************************
 * Sets the parameters needed to utilize the mayor library. This function must
 * be called before any other functions in this library can be used.
 *
 * This function does a number of things:
 * - It verifies that the user is setting up the mayor with the required
 *   parameters.
 *
 * - It sets up the internal state of the mayor library.
 *
 * - It defines the mayor's document which is needed to communicate in a CAN
 *   Kingdom system.
 *
 * - It sets up the folders for the king's document and the mayor's
 *   document.
 *
 *   @param mayor parameters for setting up the mayor.
 *
 *   @return #CK_ERR_MISSING_PARAMETER if some required parameter is not set.
 *   @return #CK_ERR_INVALID_PARAMETER if some parameter is set incorrectly.
 *   @return #CK_ERR_ITEM_NOT_FOUND if missing a requred list.
 *   @return #CK_ERR_CAPACITY_REACHED if a requred list is too small.
 *   @return #CK_OK on success.
 ******************************************************************************/
ck_err_t ck_mayor_init(const ck_mayor_t *mayor);

/*******************************************************************************
 * Parse the king's letter and act on it.
 *
 * @param letter the received king's letter.
 * @return #CK_ERR_NOT_INITIALIZED if #ck_mayor_init() has not been called.
 * @return #CK_ERR_INVALID_KINGS_LETTER if the letter is not a valid king's
 *         letter.
 * @return #CK_ERR_UNSUPPORTED_KINGS_PAGE if the letter contains a king's page
 *         not supported by this implementation.
 * @return #CK_OK on success or if the letter is not addressed to the caller.
 ******************************************************************************/
ck_err_t ck_process_kings_letter(const ck_letter_t *letter);

/*******************************************************************************
 * Adds a user-defined mayor's page to the mayor's document.
 * The function will store a pointer to the page instead of copying it.
 * Therefore, the page needs to persist in memory.
 *
 * @param page the page to add to the mayor's document.
 *
 * @return #CK_ERR_INVALID_PARAMETER if page is NULL.
 * @return #CK_ERR_CAPACITY_REACHED if the mayor's document cannot hold more
 *         pages.
 * @return #CK_OK on success.
 ******************************************************************************/
ck_err_t ck_add_mayors_page(ck_page_t *page);

#ifdef __cplusplus
}
#endif

#endif /* CK_MAYOR_H */
