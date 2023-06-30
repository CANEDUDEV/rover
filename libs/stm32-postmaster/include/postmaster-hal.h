#ifndef POSTMASTER_HAL_H
#define POSTMASTER_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f3xx_hal.h"
#include "types.h"

void postmaster_init(CAN_HandleTypeDef *hcan);
CAN_HandleTypeDef *get_can_handle(void);

// Convert HAL CAN frame to ck_letter_t.
ck_letter_t frame_to_letter(CAN_RxHeaderTypeDef *header, uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif /* POSTMASTER_HAL_H*/
