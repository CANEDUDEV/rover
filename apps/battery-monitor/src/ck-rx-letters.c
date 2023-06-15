#include "ck-rx-letters.h"

#include <string.h>

#include "adc.h"
#include "battery.h"
#include "error.h"
#include "peripherals.h"
#include "print.h"

// 7 bytes in page
//
// byte 0: jumper config.
//
//     0x00: ALL_OFF
//     0x01: X11_ON
//     0x02: X12_ON
//     0x03: ALL_ON
//     Ignore all other values.
//
// byte 1: fuse config.
//
//     0x00: FUSE_50_AMPERE
//     0x01: FUSE_100_AMPERE
//     Ignore all other values.
//
// byte 2: Set to 0x01 to manually set over-current threshold using bytes 3-6.
//         All other values will cause over-current threshold to be set to the
//         fuse config - 500mA.
//
// bytes 3-6: over-current threshold in mA, 32-bit number where byte 5 is MSB
//            and byte 2 is LSB.
//     Typical range is 0-100000 mA, but all values are accepted. Use with
//     caution, since setting it too high will result in burned fuses. Byte 2
//     needs to be set to 0x01 for this setting to have any effect.
int process_jumper_and_fuse_conf_letter(const ck_letter_t *letter) {
  if (letter->page.line_count != 7) {  // NOLINT
    return APP_NOT_OK;
  }
  set_jumper_config(letter->page.lines[0]);
  set_fuse_config(letter->page.lines[1]);
  if (letter->page.lines[2] == 0x1) {
    uint32_t over_current_threshold = 0;
    memcpy(&over_current_threshold, &letter->page.lines[3],
           sizeof(over_current_threshold));
    set_over_current_threshold(over_current_threshold);
  }
  return APP_OK;
}

// 2 bytes in page
//
// bytes 0-1: Desired voltage for the regulated out voltage port in mV. 16-bit
// number where byte 0 is MSB and byte 1 is LSB. Voltage will be regulated as
// close as possible to desired voltage.
int process_reg_out_voltage_letter(const ck_letter_t *letter) {
  if (letter->page.line_count != 2) {
    return APP_NOT_OK;
  }
  return APP_OK;
}

int process_output_on_off_letter(const ck_letter_t *letter) {
  if (letter->page.line_count != 2) {
    return APP_NOT_OK;
  }
  return APP_OK;
}

int process_report_freq_letter(const ck_letter_t *letter) {
  if (letter->page.line_count != 4) {
    return APP_NOT_OK;
  }
  return APP_OK;
}

int process_low_voltage_cutoff_letter(const ck_letter_t *letter) {
  if (letter->page.line_count != 2) {
    return APP_NOT_OK;
  }
  return APP_OK;
}
