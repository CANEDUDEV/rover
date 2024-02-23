#include "ck-rx-letters.h"

#include <string.h>

#include "battery.h"
#include "ck-data.h"
#include "error.h"
#include "freertos-tasks.h"
#include "jumpers.h"
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
int process_set_reg_out_voltage_letter(const ck_letter_t *letter) {
  ck_data_t *ck_data = get_ck_data();
  if (letter->page.line_count != ck_data->set_reg_out_voltage_folder->dlc) {
    return APP_NOT_OK;
  }

  battery_state_t *battery_state = get_battery_state();
  memcpy(&battery_state->target_reg_out_voltage, letter->page.lines,
         sizeof(battery_state->target_reg_out_voltage));

  return APP_OK;
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

  battery_state_t *battery_state = get_battery_state();

  uint8_t vbat_out = letter->page.lines[0];
  switch (vbat_out) {
    case 0:
      set_vbat_power_off();
      break;

    case 1:
      power_output_reset(&battery_state->vbat_out);
      set_vbat_power_on();
      break;

    default:
      break;
  }

  uint8_t reg_out = letter->page.lines[1];
  switch (reg_out) {
    case 0:
      set_reg_out_power_off();
      break;

    case 1:
      power_output_reset(&battery_state->reg_out);
      set_reg_out_power_on();
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
// bytes 0-1: low voltage cutoff value in mV. 16-bit unsigned integer in little
//            endian format. The value should correspond to the lowest voltage
//            each individual cell in the battery may have, not the total
//            voltage of the cells combined.
//
int process_low_voltage_cutoff_letter(const ck_letter_t *letter) {
  ck_data_t *ck_data = get_ck_data();
  if (letter->page.line_count != ck_data->low_voltage_cutoff_folder->dlc) {
    return APP_NOT_OK;
  }

  battery_cells_t *battery_cells = &get_battery_state()->cells;

  memcpy(&battery_cells->low_voltage_cutoff, letter->page.lines,
         sizeof(battery_cells->low_voltage_cutoff));

  battery_cells->low_voltage_fault = false;  // Reset

  return APP_OK;
}

// 4 bytes in page
//
// bytes 0-3: VBAT over-current threshold in mA. value is a 32-bit unsigned
//            integer in little endian format. Typical range 0-120000 mA. Use
//            with caution, since setting it too high can result in burned
//            fuses.
//
int process_vbat_out_overcurrent_threshold_letter(const ck_letter_t *letter) {
  ck_data_t *ck_data = get_ck_data();
  if (letter->page.line_count !=
      ck_data->vbat_out_overcurrent_threshold_folder->dlc) {
    return APP_NOT_OK;
  }

  power_output_t *power_output = &get_battery_state()->vbat_out;

  memcpy(&power_output->overcurrent_threshold, letter->page.lines,
         sizeof(power_output->overcurrent_threshold));

  power_output->overcurrent_fault = false;  // Reset

  return APP_OK;
}

// 4 bytes in page
//
// bytes 0-3: REG OUT over-current threshold in mA. value is a 32-bit unsigned
//            integer in little endian format. Typical range 0-8000 mA.
//
int process_reg_out_overcurrent_threshold_letter(const ck_letter_t *letter) {
  ck_data_t *ck_data = get_ck_data();
  if (letter->page.line_count !=
      ck_data->reg_out_overcurrent_threshold_folder->dlc) {
    return APP_NOT_OK;
  }

  power_output_t *power_output = &get_battery_state()->reg_out;

  memcpy(&power_output->overcurrent_threshold, letter->page.lines,
         sizeof(power_output->overcurrent_threshold));

  power_output->overcurrent_fault = false;  // Reset

  return APP_OK;
}
