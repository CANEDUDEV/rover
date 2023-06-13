/*******************************************************************************
 * @file app.h
 *
 * Provides data structures and application logic for the battery monitor
 * application.
 ******************************************************************************/
#ifndef APP_H
#define APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "adc.h"
#include "stm32f3xx_hal.h"

#define BATTERY_CELLS_MAX 6  // Max number of battery cells supported

// Values in mA.
typedef enum {
  FUSE_50_AMPERE = 50000,
  FUSE_100_AMPERE = 100000,
} fuse_config_t;

typedef struct {
  uint16_t cells[BATTERY_CELLS_MAX];
  uint16_t reg_out_current;
  uint32_t vbat_out_current;
} battery_state_t;

// Parses an adc reading and uses it to update the battery state.
void parse_adc_values(const adc_reading_t *adc_reading,
                      battery_state_t *battery_state);

// Define some charge states in mV. Based on LiPo batteries. Assumes maximum
// cell voltage of 4200 mV and a low voltage cutoff at 3200 mV.
typedef enum {
  LOW_VOLTAGE_CUTOFF = 3200,
  CHARGE_20_PERCENT = 3400,
  CHARGE_40_PERCENT = 3600,
  CHARGE_60_PERCENT = 3800,
  CHARGE_80_PERCENT = 4000,
  CHARGE_100_PERCENT = 4200,
} battery_charge_t;

uint8_t set_charge_state_led(const battery_charge_t *charge);

battery_charge_t read_battery_charge(const battery_state_t *battery_state);

#ifdef __cplusplus
}
#endif

#endif /* APP_H */
