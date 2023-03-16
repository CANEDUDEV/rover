#include "app.h"

#include <string.h>

#define ADC_REF_VOLTAGE 3300   // mV
#define ADC_MAX (1 << 12) - 1  // 12-bit ADC

#define SERVO_BASE_ID 100
#define SENSOR_POWER_MESSAGE_ID SERVO_BASE_ID + 0
#define SENSOR_POWER_MESSAGE_DLC 2

#define SERVO_CURRENT_MESSAGE_ID SERVO_BASE_ID + 1
#define SERVO_CURRENT_MESSAGE_DLC 2

#define BAT_VOLTAGE_MESSAGE_ID SERVO_BASE_ID + 2
#define BAT_VOLTAGE_MESSAGE_DLC 2

#define VCC_SERVO_VOLTAGE_MESSAGE_ID SERVO_BASE_ID + 3
#define VCC_SERVO_VOLTAGE_MESSAGE_DLC 2

#define H_BRIDGE_WINDING_CURRENT_MESSAGE_ID SERVO_BASE_ID + 4
#define H_BRIDGE_WINDING_CURRENT_MESSAGE_DLC 2

// No sensor connected, so we send the raw ADC value.
void ADCToSensorPowerMessage(uint16_t adcValue, CANFrame *frame) {
  frame->id = SENSOR_POWER_MESSAGE_ID;
  frame->dlc = SENSOR_POWER_MESSAGE_DLC;
  memcpy(frame->data, &adcValue, sizeof(adcValue));
}

/* The LT6106 current sensor specifies that
 * Isense = Vout * Rin / (Rsense * Rout).
 * Vout is measured by ADC, Vout = ADC_REF_VOLTAGE * adcValue / ADC_MAX
 *
 * Resistances for Servo Node rev E:
 * Rin = R1 = 51 Ohm
 * Rsense = R2 = 0.002 Ohm
 * Rout = R3 = 5100 Ohm
 */
void ADCToServoCurrentMessage(uint16_t adcValue, CANFrame *frame) {
  frame->id = SERVO_CURRENT_MESSAGE_ID;
  frame->dlc = SERVO_CURRENT_MESSAGE_DLC;

  const uint32_t Vout = (ADC_REF_VOLTAGE * adcValue) / ADC_MAX;  // Vout in mV
  const uint32_t Rin = 51;
  const uint32_t Rout = 5100;
  const uint32_t Rsense = 2;  // use mOhm here, this counteracts mV value in
                              // Vout and avoids floating point

  const uint16_t Isense =
      (uint16_t)(1000 * Vout * Rin / (Rsense * Rout));  // in mA

  memcpy(frame->data, &Isense, sizeof(Isense));
}

/* Voltage divider with R1 = 47kOhm and R2 = 6.2kOhm
 * Vbat = Vout * (R1 + R2) / R2
 * Vout is measured by ADC, Vout = ADC_REF_VOLTAGE * adcValue / ADC_MAX
 */
void ADCToBatteryVoltageMessage(uint16_t adcValue, CANFrame *frame) {
  frame->id = BAT_VOLTAGE_MESSAGE_ID;
  frame->dlc = BAT_VOLTAGE_MESSAGE_DLC;

  // Voltages in mV
  const uint32_t Vout = (ADC_REF_VOLTAGE * adcValue) / ADC_MAX;
  const uint16_t Vbat = (uint16_t)(Vout * (47000 + 6200) / 6200);
  memcpy(frame->data, &Vbat, sizeof(adcValue));
}

/* Voltage divider with R1 = 39kOhm and R2 = 15kOhm
 * Vbat = Vout * (R1 + R2) / R2
 * Vout is measured by ADC, Vout = ADC_REF_VOLTAGE * adcValue / ADC_MAX
 */
void ADCToVCCServoVoltageMessage(uint16_t adcValue, CANFrame *frame) {
  frame->id = VCC_SERVO_VOLTAGE_MESSAGE_ID;
  frame->dlc = VCC_SERVO_VOLTAGE_MESSAGE_DLC;
  const uint32_t Vout = (ADC_REF_VOLTAGE * adcValue) / ADC_MAX;  // Vout in mV
  const uint16_t Vbat =
      (uint16_t)(Vout * (39000 + 15000) / 15000);  // Vbat in mV
  memcpy(frame->data, &Vbat, sizeof(adcValue));
}

/* DRV8801 datasheet specifies the max output current as 2.8A.
 * Specified relationship between VPROPI and Vsense: Vsense = VPROPI / 5
 * Thus, Isense = Vsense / Rsense = VPROPI / (Rsense * 5)
 * VPROPI is measured by ADC, VPROPI = ADC_REF_VOLTAGE * adcValue / ADC_MAX
 * Rsense = R33 = 0.2 Ohm (from servo node rev E schematic)
 * To simplify, Isense = VPROPI / (0.2*5) = VPROPI
 */
void ADCToHBridgeWindingCurrentMessage(uint16_t adcValue, CANFrame *frame) {
  frame->id = H_BRIDGE_WINDING_CURRENT_MESSAGE_ID;
  frame->dlc = H_BRIDGE_WINDING_CURRENT_MESSAGE_DLC;

  // Currents in mA
  const uint16_t maxCurrent = 2800;
  const uint16_t Isense = (uint16_t)((ADC_REF_VOLTAGE * adcValue) / ADC_MAX);

  uint16_t current = Isense;
  if (current > maxCurrent) {
    current = maxCurrent;
  }
  memcpy(frame->data, &Isense, sizeof(adcValue));
}
