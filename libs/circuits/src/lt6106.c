#include "lt6106.h"

/*
 * The LT6106 current sensor specifies that
 * i_sense = output_voltage * r_in / (r_sense * r_out).
 */
float lt6106_sense_current(float output_voltage,
                           const lt6106_current_sensor_t *sensor) {
  // Input offset voltage correction (typical 150 microvolts)
  const float error = 0.15F * sensor->r_out / sensor->r_in;
  return (output_voltage - error) * sensor->r_in /
         (sensor->r_sense * sensor->r_out);
}
