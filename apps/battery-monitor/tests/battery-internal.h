#ifndef BATTERY_INTERNAL_H
#define BATTERY_INTERNAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "adc.h"

void handle_faults(void);
void update_battery_cells(const adc_reading_t *adc_reading);
void update_battery_charge(void);
void update_battery_leds(void);
bool is_low_voltage_fault(void);
bool is_over_current_fault(void);
uint16_t *get_lowest_cell(void);

#ifdef __cplusplus
}
#endif

#endif  // BATTERY_INTERNAL_H
