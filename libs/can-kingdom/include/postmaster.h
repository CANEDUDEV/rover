/*******************************************************************************
 * @file postmaster.h
 *
 * The postmaster is responsible for handling CAN communication in a city. The
 * postmaster implementation is hardware dependent. This header file contains
 * the interface that needs to be fulfilled by the implementation.
 *
 * For all functions, the specified return codes should be provided by the
 * implementation.
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
 *
 * @return #CK_ERR_SEND_FAILED if letter failed to send.
 *
 * @return #CK_OK if letter is sent correctly.
 ******************************************************************************/
ck_err_t ck_send_letter(const ck_letter_t *letter);

/*******************************************************************************
 * Apply the requested communication mode in hardware.
 *
 * This function shouldn't be called directly by the user application, use
 * ck_set_comm_mode() instead.
 *
 * @param mode the desired communication mode.
 *
 * @return #CK_ERR_SET_MODE_FAILED if failed setting the communication mode.
 * @return #CK_ERR_INVALID_COMM_MODE if the requested communication mode is
 *         invalid.
 * @return #CK_OK on success.
 ******************************************************************************/
ck_err_t ck_apply_comm_mode(ck_comm_mode_t mode);

/*******************************************************************************
 * Return the ck_can_bit_timing_t prescaler that will give a bit rate of
 * 125 Kbit/s on the target hardware.
 ******************************************************************************/
uint8_t ck_get_125kbit_prescaler(void);

/*******************************************************************************
 * Set the given bit timing parameters for CAN communication.
 *
 * This function must never apply invalid CAN settings on the peripheral. Doing
 * so will make the city unable to communicate. At the very least, in the event
 * of bad settings, the hardware should revert to the old settings.
 *
 * @param bit_timing the requested ck_can_bit_timing_t.
 *
 * @return #CK_ERR_INVALID_CAN_BIT_TIMING if the bit_timing is incompatible with
 *         the target hardware.
 *
 * @return #CK_ERR_PERIPHERAL if setting the bit_timing failed for some other
 *         reason.
 *
 * @return #CK_OK on success
 ******************************************************************************/
ck_err_t ck_set_bit_timing(const ck_can_bit_timing_t *bit_timing);

/*******************************************************************************
 * Save the given bit_timing parameters to persistent storage.
 *
 * @param bit_timing the ck_can_bit_timing_t to save.
 *
 * @return #CK_ERR_PERIPHERAL if it wasn't possible to save the parameters.
 * @return #CK_OK on success
 ******************************************************************************/
ck_err_t ck_save_bit_timing(const ck_can_bit_timing_t *bit_timing);

/*******************************************************************************
 * Load a bit timing parameters from persistent storage.
 *
 * @param bit_timing the loaded bit timing parameters.
 *
 * @return #CK_ERR_PERIPHERAL if it wasn't possible to load the parameters.
 * @return #CK_OK on success
 ******************************************************************************/
ck_err_t ck_load_bit_timing(ck_can_bit_timing_t *bit_timing);

#ifdef __cplusplus
}
#endif

#endif /* CK_POSTMASTER_H */
