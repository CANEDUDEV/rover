#include "ck-rx-letters.h"

#include <math.h>
#include <string.h>

#include "battery.h"
#include "ck-data.h"
#include "error.h"
#include "freertos-tasks.h"
#include "jumpers.h"
#include "potentiometer.h"
#include "power.h"

// 1 bytes in page
//
// byte 0: current measure jumper config.
//
//     0x00: ALL_OFF
//     0x01: X11_ON
//     0x02: X12_ON
//     0x03: ALL_ON
//     Ignore all other values.
//
// byte 2: Set to 0x01 to manually set over-current threshold using bytes 3-6.
//         All other values will cause over-current threshold to be set to the
//         fuse config - 500mA.
//
int process_jumper_config_letter(const ck_letter_t *letter) {
  ck_data_t *ck_data = get_ck_data();
  if (letter->page.line_count != ck_data->jumper_config_folder->dlc) {
    return APP_NOT_OK;
  }

  set_current_measure_jumper_config(letter->page.lines[0]);

  return APP_OK;
}

// 2 bytes in page
//
// bytes 0-1: desired output voltage on the regulated output.
//
// Voltage regulator (PTN78020W) formula for calculating the output voltage
// using a given resistance:
//
// R_set = 54.9 kOhm * (1.25 V / (V_o - 2.5 V)) - 6.490 kOhm,
//
// Where V_o is the target voltage and R_set is the required resistance to set
// the target_voltage.
//
// We use a potentiometer to set R_set. The potentiometer (TPL0102-100) takes a
// value between 0 and 255 (total 256 steps) corresponding to resistances
// between 0 and 100 kOhm. This gives the formula
//  R_set = (100 kOhm / 256) * potentiometer_value =>
//  potentiometer_value = (256 / 100 kOhm) * R_set
//
int process_set_reg_out_voltage_letter(const ck_letter_t *letter) {
  ck_data_t *ck_data = get_ck_data();
  if (letter->page.line_count != ck_data->set_reg_out_voltage_folder->dlc) {
    return APP_NOT_OK;
  }

  uint16_t target_voltage = 0;
  memcpy(&target_voltage, letter->page.lines, sizeof(target_voltage));

  const uint16_t min_voltage = 3300;   // CPU voltage
  const uint16_t max_voltage = 12600;  // PTN78020W regulator max
  if (target_voltage < min_voltage || target_voltage > max_voltage) {
    return APP_NOT_OK;
  }

  // Voltages in mV
  const float r_set = 54900 * (1250 / ((float)target_voltage - 2500)) - 6490;
  const float potentiometer_f = r_set * (256.0F / (100 * 1000));
  uint8_t potentiometer_value = (uint8_t)roundf(potentiometer_f);

  // Check for overflow
  if (potentiometer_f > 0xFF) {  // NOLINT
    potentiometer_value = 0xFF;  // NOLINT
  }
  if (potentiometer_f < 0) {
    potentiometer_value = 0;
  }

  return write_potentiometer_value(potentiometer_value);
}

// 2 bytes in page
//
// byte 0: reg out on/off, set to 0 for OFF, 1 for ON, all other values are
// ignored byte 1: power on/off, set to 0 for OFF, 1 for ON, all other values
// are ignored
int process_output_on_off_letter(const ck_letter_t *letter) {
  ck_data_t *ck_data = get_ck_data();
  if (letter->page.line_count != ck_data->output_on_off_folder->dlc) {
    return APP_NOT_OK;
  }

  uint8_t vbat_out = letter->page.lines[0];
  switch (vbat_out) {
    case 0:
      set_vbat_power_off();
      break;

    case 1:
      battery_state_reset();
      set_vbat_power_on();
      break;

    default:
      break;
  }

  uint8_t reg_out = letter->page.lines[1];
  switch (reg_out) {
    case 0:
      set_reg_vout_power_off();
      break;

    case 1:
      set_reg_vout_power_on();
      break;

    default:
      break;
  }

  return APP_OK;
}

// 2 bytes in page
//
// bytes 0-1: battery reporting period in ms, i.e. how often to send
// measurements over CAN.
int process_report_freq_letter(const ck_letter_t *letter) {
  ck_data_t *ck_data = get_ck_data();
  if (letter->page.line_count != ck_data->report_freq_folder->dlc) {
    return APP_NOT_OK;
  }

  task_periods_t periods;

  memcpy(&periods.battery_report_period_ms, letter->page.lines,
         sizeof(periods.battery_report_period_ms));

  set_task_periods(&periods);

  return APP_OK;
}

// 2 bytes in page
//
// bytes 0-1: low voltage cutoff value in mV. 16-bit number where byte 0 is LSB
// and byte 1 is MSB. The value should correspond to the lowest voltage each
// individual cell in the battery may have, not the total voltage of the cells
// combined.
int process_low_voltage_cutoff_letter(const ck_letter_t *letter) {
  ck_data_t *ck_data = get_ck_data();
  if (letter->page.line_count != ck_data->low_voltage_cutoff_folder->dlc) {
    return APP_NOT_OK;
  }

  battery_state_t *battery_state = get_battery_state();

  memcpy(&battery_state->low_voltage_cutoff, letter->page.lines,
         sizeof(battery_state->low_voltage_cutoff));

  return APP_OK;
}

// 4 bytes in page
//
// bytes 0-3: over-current threshold in mA, 32-bit number where byte 3 is MSB
//            and byte 0 is LSB.
//
//  Typical range is 0-100000 mA, but all values are accepted. Use with
//  caution, since setting it too high will result in burned fuses.
//
int process_over_current_threshold_letter(const ck_letter_t *letter) {
  ck_data_t *ck_data = get_ck_data();
  if (letter->page.line_count != ck_data->over_current_threshold_folder->dlc) {
    return APP_NOT_OK;
  }

  uint32_t over_current_threshold = 0;
  memcpy(&over_current_threshold, letter->page.lines,
         sizeof(over_current_threshold));
  set_over_current_threshold(over_current_threshold);

  return APP_OK;
}
