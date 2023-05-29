#include "postmaster-hal.h"

static CAN_HandleTypeDef *hcan;

void postmaster_init(CAN_HandleTypeDef *_hcan) { hcan = _hcan; }

CAN_HandleTypeDef *get_can_handle(void) { return hcan; }
