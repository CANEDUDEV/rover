#ifndef BATTERY_H
#define BATTERY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "adc.h"

#define BATTERY_CELLS_MAX 6  // Max number of battery cells supported
// Cells that don't exist will report a value close to 0. We set a cell
// detection threshold voltage at 100 mV.
#define BATTERY_CELL_DETECTION_THRESHOLD 100

#define BATTERY_CHARGE_100_PERCENT 100

#define LIPO_CELL_MIN_VOLTAGE 3200
#define LIPO_CELL_MAX_VOLTAGE 4200

typedef struct {
  uint16_t cell_min_voltage;
  uint16_t cell_max_voltage;
  uint16_t cell_voltage[BATTERY_CELLS_MAX];

  uint16_t reg_out_current;
  uint16_t reg_out_voltage;
  uint16_t vbat_out_voltage;
  uint32_t vbat_out_current;
  uint8_t charge;
  volatile bool over_current_fault;  // Set to true by GPIO external interrupt
                                     // on the OVER_CURRENT pin.
  bool low_voltage_fault;

  uint32_t over_current_threshold;  // At what value the over_current_fault will
                                    // trigger from software. Defaults to fuse
                                    // config - 500mA.

  uint16_t low_voltage_cutoff;  // In mV.
} battery_state_t;

// Values in mA.
typedef enum {
  FUSE_50_AMPERE = 50000,
  FUSE_100_AMPERE = 100000,
} fuse_config_t;

void battery_state_init(void);
void battery_state_reset(void);
battery_state_t *get_battery_state(void);
void set_fuse_config(fuse_config_t fuse_config);
void set_over_current_threshold(uint32_t threshold);
// Parses an adc reading and uses it to update the battery state.
void update_battery_state(const adc_reading_t *adc_reading);

#ifdef __cplusplus
}
#endif

#endif /* BATTERY_H */
