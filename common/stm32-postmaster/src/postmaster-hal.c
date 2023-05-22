#include "postmaster-hal.h"

static CAN_HandleTypeDef *hcan;
static ck_comm_mode_t comm_mode;

void postmaster_init(CAN_HandleTypeDef *_hcan) { hcan = _hcan; }

CAN_HandleTypeDef *get_can_handle(void) { return hcan; }

ck_comm_mode_t get_comm_mode(void) { return comm_mode; }
void set_comm_mode(ck_comm_mode_t mode) { comm_mode = mode; }
