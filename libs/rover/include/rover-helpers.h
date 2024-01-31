#ifndef ROVER_HELPERS_H
#define ROVER_HELPERS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "types.h"

bool someone_else_is_king(void);
ck_err_t send_default_letter(void);
ck_err_t set_rover_base_number(void);
ck_err_t assign_rover_envelopes(const ck_id_t *own_id);
ck_err_t configure_rover_settings(void);
ck_err_t start_communication(void);

#ifdef __cplusplus
}
#endif

#endif  // ROVER_HELPERS_H
