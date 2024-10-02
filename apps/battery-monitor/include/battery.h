#ifndef BATTERY_H
#define BATTERY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "adc.h"

// Max number of battery cells supported
#define BATTERY_CELLS_MAX 6
// Cells that don't exist will report a value close to 0. We set a cell
// detection threshold voltage at 100 mV.
#define BATTERY_CELL_DETECTION_THRESHOLD_MV 100

#define BATTERY_CHARGE_100_PERCENT 100

#define LIPO_CELL_MIN_VOLTAGE_MV 3000
#define LIPO_CELL_MAX_VOLTAGE_MV 4200

#define DEFAULT_LOW_VOLTAGE_CUTOFF_MV LIPO_CELL_MIN_VOLTAGE_MV

#define DEFAULT_REG_OUT_VOLTAGE_LOW_MV 5100
#define DEFAULT_REG_OUT_VOLTAGE_HIGH_MV 12100

#define DEFAULT_OVERCURRENT_THRESHOLD_MA (95 * 1000)
#define DEFAULT_REG_OUT_OVERCURRENT_THRESHOLD_MA (8 * 1000)

typedef struct {
  uint32_t voltage;
  uint32_t current;

  uint32_t overcurrent_threshold;
  volatile bool overcurrent_fault;  // Accessed by GPIO external interrupt
                                    // on the OVER_CURRENT pin.

} power_output_t;

typedef struct {
  uint16_t min_voltage;
  uint16_t max_voltage;
  uint16_t voltage[BATTERY_CELLS_MAX];

  uint16_t low_voltage_cutoff;
  bool low_voltage_fault;

} battery_cells_t;

typedef struct {
  battery_cells_t cells;
  power_output_t vbat_out;
  power_output_t reg_out;

  uint8_t charge;
  uint32_t target_reg_out_voltage;

} battery_state_t;

battery_state_t *get_battery_state(void);

void battery_state_init(void);
void battery_state_reset(void);
void power_output_reset(power_output_t *output);

// Parses an adc reading and uses it to update the battery state.
void update_battery_state(const adc_reading_t *adc_reading);

#ifdef __cplusplus
}
#endif

#endif /* BATTERY_H */
