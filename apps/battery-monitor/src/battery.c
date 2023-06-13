#include "battery.h"

#include <string.h>

#include "adc.h"
#include "ck-data.h"
#include "led.h"
#include "peripherals.h"
#include "ports.h"
#include "potentiometer.h"

// STM32 Common
#include "print.h"

// CK
#include "mayor.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

static battery_state_t battery_state;

void update_battery_charge(void);
void update_battery_leds(void);
void send_docs(void);
charge_t lowest_cell(const charge_t *cell_charge);

void battery_state_init(void) {
  for (int i = 0; i < BATTERY_CELLS_MAX; i++) {
    battery_state.cells[i] = 0;
  }
  battery_state.reg_out_current = 0;
  battery_state.vbat_out_current = 0;
  battery_state.charge = CHARGE_100_PERCENT;
  battery_state.over_current_fault = false;
}

battery_state_t *get_battery_state(void) { return &battery_state; }

// NOLINTBEGIN(*-magic-numbers)
void update_battery_state(const adc_reading_t *adc_reading) {
  for (int i = 0; i < 4; i++) {
    battery_state.cells[i] = adc_to_cell_voltage(adc_reading->adc1_buf[i]);
  }
  battery_state.cells[4] = adc_to_cell_voltage(adc_reading->adc2_buf[0]);
  battery_state.cells[5] = adc_to_cell_voltage(adc_reading->adc2_buf[1]);
  battery_state.reg_out_current =
      adc_to_reg_out_current(adc_reading->adc2_buf[2]);
  battery_state.vbat_out_current =
      adc_to_vbat_out_current(adc_reading->adc2_buf[3]);

  update_battery_charge();
  update_battery_leds();
}
// NOLINTEND(*-magic-numbers)

// Always report lowest detected charge to detect the most discharged cell.
void update_battery_charge(void) {
  charge_t cell_charges[BATTERY_CELLS_MAX];
  for (int i = 0; i < BATTERY_CELLS_MAX; i++) {
    // If cell is not connected, we report it as fully charged to not use its
    // values in the low voltage detection logic.
    if (battery_state.cells[i] < BATTERY_CELL_DETECTION_THRESHOLD) {
      cell_charges[i] = CHARGE_100_PERCENT;
      continue;
    }

    if (battery_state.cells[i] <= LOW_VOLTAGE_CUTOFF) {
      cell_charges[i] = LOW_VOLTAGE_CUTOFF;
    } else if (battery_state.cells[i] <= CHARGE_20_PERCENT) {
      cell_charges[i] = CHARGE_20_PERCENT;
    } else if (battery_state.cells[i] <= CHARGE_40_PERCENT) {
      cell_charges[i] = CHARGE_40_PERCENT;
    } else if (battery_state.cells[i] <= CHARGE_60_PERCENT) {
      cell_charges[i] = CHARGE_60_PERCENT;
    } else if (battery_state.cells[i] <= CHARGE_80_PERCENT) {
      cell_charges[i] = CHARGE_80_PERCENT;
    } else {
      cell_charges[i] = CHARGE_100_PERCENT;
    }
  }

  // Only update state if lower than previous, to avoid blinking leds when
  // somewhere in-between charge states.
  charge_t lowest = lowest_cell(cell_charges);
  if (lowest < battery_state.charge) {
    battery_state.charge = lowest;
  }
}

void update_battery_leds(void) {
  switch (battery_state.charge) {
    case CHARGE_100_PERCENT:
      set_led_color(LED6, GREEN);
      set_led_color(LED7, GREEN);
      break;
    case CHARGE_80_PERCENT:
      set_led_color(LED6, GREEN);
      set_led_color(LED7, NONE);
      break;
    case CHARGE_60_PERCENT:
      set_led_color(LED6, ORANGE);
      set_led_color(LED7, ORANGE);
      break;
    case CHARGE_40_PERCENT:
      set_led_color(LED6, ORANGE);
      set_led_color(LED7, NONE);
      break;
    case CHARGE_20_PERCENT:
      set_led_color(LED6, RED);
      set_led_color(LED7, RED);
      break;
    case LOW_VOLTAGE_CUTOFF:
    default:
      break;
  }
}

charge_t lowest_cell(const charge_t *cell_charge) {
  charge_t lowest = CHARGE_100_PERCENT;
  for (int i = 0; i < BATTERY_CELLS_MAX; i++) {
    // Comparison based on enum ordering
    if (cell_charge[i] < lowest) {
      lowest = cell_charge[i];
    }
  }
  return lowest;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN) {
  if (GPIO_PIN != OVER_CURRENT_PIN) {
    return;
  }
  battery_state.over_current_fault = true;
}
