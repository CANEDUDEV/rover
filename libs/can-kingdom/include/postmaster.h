/*******************************************************************************
 * @file postmaster.h
 *
 * The postmaster is responsible for handling CAN communication in a city. The
 * postmaster implementation is hardware dependent. This header file contains
 * the interface that needs to be fulfilled by the implementation.
 ******************************************************************************/

#ifndef CK_POSTMASTER_H
#define CK_POSTMASTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "types.h"

/*******************************************************************************
 * Try to send a letter.
 *
 * This function should not be called directly by the user applicaiton, use
 * ck_send_document() or ck_send_mayors_page() instead.
 *
 * @param letter the letter to send.
 * @param dlc the DLC of the folder which the letter is constructed from.
 *
 * @return #CK_ERR_SEND_FAILED if letter failed to send.
 * @return #CK_ERR_INVALID_CAN_DLC if the dlc is out of bounds.
 *
 * @return #CK_OK if letter is sent correctly.
 ******************************************************************************/
ck_err_t ck_send_letter(const ck_letter_t *letter, uint8_t dlc);

/*******************************************************************************
 * Apply the requested communication mode in hardware. This function shouldn't
 * be called from user applications, use ck_set_comm_mode() instead.
 *
 * @param mode the desired communication mode.
 *
 * @return #CK_OK on success.
 * @return #CK_ERR_SET_MODE_FAILED if failed setting the communication mode.
 * @return #CK_ERR_INVALID_COMM_MODE if the requested communication mode is
 *         invalid.
 ******************************************************************************/
ck_err_t ck_apply_comm_mode(ck_comm_mode_t mode);

#ifdef __cplusplus
}
#endif

#endif /* CK_POSTMASTER_H */
