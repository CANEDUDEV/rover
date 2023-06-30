#include "adc.h"

#define ADC_REF_VOLTAGE 3300     // mV
#define ADC_MAX ((1 << 12) - 1)  // 12-bit ADC

// No sensor connected, so we send the raw ADC value.
uint16_t adc_to_sensor_power(uint16_t adc_value) { return adc_value; }

/* The LT6106 current sensor specifies that
 * i_sense = v_out * r_in / (r_sense * r_out).
 * v_out is measured by ADC, v_out = ADC_REF_VOLTAGE * adc_value / ADC_MAX
 *
 * Resistances for servo board rev. E:
 * r_in = R1 = 51 Ohm
 * r_sense = R2 = 0.002 Ohm
 * r_out = R3 = 5100 Ohm
 */
uint16_t adc_to_servo_current(uint16_t adc_value) {
  uint32_t v_out = (ADC_REF_VOLTAGE * adc_value) / ADC_MAX;  // v_out in mV
  const uint32_t r_in = 51;
  const uint32_t r_out = 5100;
  const uint32_t r_sense = 2;  // use mOhm here, this counteracts mV value in
                               // v_out and avoids floating point

  const uint32_t i_sense = 1000 * v_out * r_in / (r_sense * r_out);  // in mA

  return (uint16_t)i_sense;
}

/* Voltage divider with R1 = 47kOhm and R2 = 6.2kOhm
 * v_battery = v_out * (R1 + R2) / R2
 * v_out is measured by ADC, v_out = ADC_REF_VOLTAGE * adc_value / ADC_MAX
 */
uint16_t adc_to_battery_voltage(uint16_t adc_value) {
  // Voltages in mV
  uint32_t v_out = (ADC_REF_VOLTAGE * adc_value) / ADC_MAX;
  const uint32_t v_battery = v_out * (47000 + 6200) / 6200;
  return (uint16_t)v_battery;
}

/* Voltage divider with R1 = 39kOhm and R2 = 15kOhm
 * v_servo = v_out * (R1 + R2) / R2
 * v_out is measured by ADC, v_out = ADC_REF_VOLTAGE * adc_value / ADC_MAX
 */
uint16_t adc_to_servo_voltage(uint16_t adc_value) {
  uint32_t v_out = (ADC_REF_VOLTAGE * adc_value) / ADC_MAX;  // v_out in mV
  const uint32_t v_servo = v_out * (39000 + 15000) / 15000;  // v_battery in mV
  return (uint16_t)v_servo;
}

/* DRV8801 datasheet specifies the max output current as 2.8A.
 * Specified relationship between VPROPI and Vsense: Vsense = VPROPI / 5
 * Thus, i_sense = Vsense / r_sense = VPROPI / (r_sense * 5)
 * VPROPI is measured by ADC, VPROPI = ADC_REF_VOLTAGE * adc_value / ADC_MAX
 * r_sense = R33 = 0.2 Ohm (from servo board rev. E schematic)
 * To simplify, i_sense = VPROPI / (0.2*5) = VPROPI
 */
uint16_t adc_to_h_bridge_current(uint16_t adc_value) {
  // Currents in mA
  const uint16_t max_current = 2800;
  const uint32_t i_sense = (ADC_REF_VOLTAGE * adc_value) / ADC_MAX;

  uint16_t current = i_sense;
  if (current > max_current) {
    current = max_current;
  }
  return (uint16_t)current;
}
