/*
 * The CAN Kingdom post office is responsible for handling CAN messages. The
 * post office code is architecture dependent. This file contains the
 * interface that needs to be fulfilled by the implementation.
 */

#ifndef CK_POSTMASTER_H
#define CK_POSTMASTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "types.h"

ck_err_t ck_send_letter(const ck_letter_t *letter);

#ifdef __cplusplus
}
#endif

#endif /* CK_POSTMASTER_H */
