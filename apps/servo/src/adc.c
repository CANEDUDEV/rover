#include "adc.h"

#include "math.h"

#define ADC_REF_VOLTAGE 3300            // mV
#define ADC_RESOLUTION ((1 << 12) - 1)  // 12-bit ADC

/* Convert the measured servo position sensor value to an angle.
 *
 * 0 mV = 0 deg, 3300 mV = 360 deg. Neutral is 180 deg, so we use it as base.
 * Outputs -180 to 180 degrees.
 */
int16_t adc_to_servo_position(uint16_t adc_value) {
  const float angle = 360.0F * (float)adc_value / ADC_RESOLUTION - 180;
  return (int16_t)roundf(angle);
}

/* The LT6106 current sensor specifies that
 * i_sense = v_out * r_in / (r_sense * r_out).
 * v_out is measured by ADC, v_out = ADC_REF_VOLTAGE * adc_value /
 * ADC_RESOLUTION
 *
 * Resistances for servo board rev. E:
 * r_in = R1 = 51 Ohm
 * r_sense = R2 = 0.002 Ohm
 * r_out = R3 = 5100 Ohm
 */
uint16_t adc_to_servo_current(uint16_t adc_value) {
  float v_out =
      ADC_REF_VOLTAGE * adc_value / (float)ADC_RESOLUTION;  // v_out in mV
  const float r_in = 51;
  const float r_out = 5100;
  const float r_sense = 2;  // use mOhm here, this counteracts mV value in
                            // v_out and avoids floating point

  const float i_sense = 1000 * v_out * r_in / (r_sense * r_out);  // in mA

  return (uint16_t)roundf(i_sense);
}

/* Voltage divider with R1 = 47kOhm and R2 = 6.2kOhm
 * v_in = v_out * (R1 + R2) / R2
 * v_out is measured by ADC, v_out = ADC_REF_VOLTAGE * adc_value /
 * ADC_RESOLUTION
 * To get v_battery we need to add the voltage drop of the Schottky diode,
 * around 300 mV => v_battery = v_in + 300.
 */
uint16_t adc_to_battery_voltage(uint16_t adc_value) {
  // Voltages in mV
  float v_out = ADC_REF_VOLTAGE * adc_value / (float)ADC_RESOLUTION;
  const float v_battery = v_out * (47000 + 6200) / 6200 + 300;
  return (uint16_t)roundf(v_battery);
}

/* Voltage divider with R1 = 39kOhm and R2 = 15kOhm
 * v_servo = v_out * (R1 + R2) / R2
 * v_out is measured by ADC, v_out = ADC_REF_VOLTAGE * adc_value /
 * ADC_RESOLUTION
 */
uint16_t adc_to_servo_voltage(uint16_t adc_value) {
  float v_out =
      ADC_REF_VOLTAGE * adc_value / (float)ADC_RESOLUTION;  // v_out in mV
  const float v_servo = v_out * (39000 + 15000) / 15000;    // v_battery in mV
  return (uint16_t)roundf(v_servo);
}

/* DRV8801 datasheet specifies the max output current as 2.8A.
 * Specified relationship between VPROPI and Vsense: Vsense = VPROPI / 5
 * Thus, i_sense = Vsense / r_sense = VPROPI / (r_sense * 5)
 * VPROPI is measured by ADC, VPROPI = ADC_REF_VOLTAGE * adc_value /
 * ADC_RESOLUTION r_sense = R33 = 0.2 Ohm (from servo board rev. E schematic) To
 * simplify, i_sense = VPROPI / (0.2*5) = VPROPI
 */
uint16_t adc_to_h_bridge_current(uint16_t adc_value) {
  // Currents in mA
  const uint16_t max_current = 2800;
  const float i_sense = ADC_REF_VOLTAGE * adc_value / (float)ADC_RESOLUTION;

  uint16_t current = (uint16_t)roundf(i_sense);
  if (current > max_current) {
    current = max_current;
  }
  return (uint16_t)current;
}
