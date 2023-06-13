#ifndef BATTERY_H
#define BATTERY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "adc.h"

// Cells that don't exist will report a value close to 0. We set a cell
// detection threshold voltage at 100 mV.
#define BATTERY_CELL_DETECTION_THRESHOLD 100
#define BATTERY_CELLS_MAX 6  // Max number of battery cells supported

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

typedef struct {
  uint16_t cells[BATTERY_CELLS_MAX];
  uint16_t reg_out_current;
  uint32_t vbat_out_current;
} battery_state_t;

// Values in mA.
typedef enum {
  FUSE_50_AMPERE = 50000,
  FUSE_100_AMPERE = 100000,
} fuse_config_t;

battery_charge_t read_battery_charge(const battery_state_t *battery_state);
uint8_t update_battery_leds(const battery_charge_t *charge);

// Parses an adc reading and uses it to update the battery state.
void update_battery_state(const adc_reading_t *adc_reading,
                          battery_state_t *battery_state);

#ifdef __cplusplus
}
#endif

#endif /* BATTERY_H */
