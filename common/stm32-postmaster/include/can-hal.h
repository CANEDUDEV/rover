#ifndef STM32_POSTMASTER_CAN_HAL_H
#define STM32_POSTMASTER_CAN_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f3xx_hal.h"
#include "types.h"

void postmaster_init(CAN_HandleTypeDef *hcan);
CAN_HandleTypeDef *get_can_handle(void);

ck_comm_mode_t get_comm_mode(void);
void set_comm_mode(ck_comm_mode_t mode);

#ifdef __cplusplus
}
#endif

#endif /* STM32_POSTMASTER_CAN_HAL_H */
