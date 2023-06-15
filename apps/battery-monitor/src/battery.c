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
  set_fuse_config(FUSE_100_AMPERE);
}

battery_state_t *get_battery_state(void) { return &battery_state; }

void set_fuse_config(fuse_config_t fuse_config) {
  switch (fuse_config) {
    case FUSE_50_AMPERE:
      set_over_current_threshold(FUSE_50_AMPERE - 500);  // NOLINT
      break;

    case FUSE_100_AMPERE:
      set_over_current_threshold(FUSE_100_AMPERE - 500);  // NOLINT
      break;

    default:  // Ignore
      break;
  }
}

// We might detect false positives if the jumper config is set to detect low
// currents but the actual currents are much higher. Nevertheless, the largest
// part of the total current should be the vbat_out_current.
//
// TODO: calculate the maximum measured reg_out_current based on the jumper
// config, use it to set a better over_current_threshold that considers the
// reg_out_current separately from the vbat_out_current.
void set_over_current_threshold(uint32_t threshold) {
  battery_state.over_current_threshold = threshold;
}

void update_battery_state(const adc_reading_t *adc_reading) {
  for (int i = 0; i < 4; i++) {
    battery_state.cells[i] = adc_to_cell_voltage(adc_reading->adc1_buf[i]);
  }
  // NOLINTBEGIN(*-magic-numbers)
  battery_state.cells[4] = adc_to_cell_voltage(adc_reading->adc2_buf[0]);
  battery_state.cells[5] = adc_to_cell_voltage(adc_reading->adc2_buf[1]);
  // NOLINTEND(*-magic-numbers)
  battery_state.reg_out_current =
      adc_to_reg_out_current(adc_reading->adc2_buf[2]);
  battery_state.vbat_out_current =
      adc_to_vbat_out_current(adc_reading->adc2_buf[3]);

  update_battery_charge();
  update_battery_leds();

  // Check if over current or low voltage protection has triggered.
  uint32_t current =
      battery_state.reg_out_current + battery_state.vbat_out_current;
  if (battery_state.charge == LOW_VOLTAGE_CUTOFF ||
      battery_state.over_current_fault ||
      current > battery_state.over_current_threshold) {
    // Turn off the power outputs to reduce the battery power drain.
    HAL_GPIO_WritePin(REG_PWR_ON_GPIO_PORT, REG_PWR_ON_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(POWER_OFF_GPIO_PORT, POWER_OFF_PIN, GPIO_PIN_RESET);
    // Blink LEDs red to show user something is wrong.
    blink_leds_red();
  }
}

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

    case LOW_VOLTAGE_CUTOFF:  // Ignore, LED should be handled elsewhere
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
