#include "app.h"

#include <string.h>

#include "FreeRTOS.h"
#include "main.h"

#define ADC_REF_VOLTAGE 3300     // mV
#define ADC_MAX ((1 << 12) - 1)  // 12-bit ADC

#define SERVO_BASE_ID 100
#define SENSOR_POWER_MESSAGE_ID (SERVO_BASE_ID + 0)
#define SENSOR_POWER_MESSAGE_DLC 2

#define SERVO_CURRENT_MESSAGE_ID (SERVO_BASE_ID + 1)
#define SERVO_CURRENT_MESSAGE_DLC 2

#define BAT_VOLTAGE_MESSAGE_ID (SERVO_BASE_ID + 2)
#define BAT_VOLTAGE_MESSAGE_DLC 2

#define VCC_SERVO_VOLTAGE_MESSAGE_ID (SERVO_BASE_ID + 3)
#define VCC_SERVO_VOLTAGE_MESSAGE_DLC 2

#define H_BRIDGE_WINDING_CURRENT_MESSAGE_ID (SERVO_BASE_ID + 4)
#define H_BRIDGE_WINDING_CURRENT_MESSAGE_DLC 2

#define POT_ADDR (0x50 << 1)  // Shift left to match STM32 specification
#define POT_IVRA_ADDR 0x0     // VCC_Servo potentiometer
#define POT_IVRB_ADDR 0x1     // VDD_Sensor potentiometer

#define POT_IVRA_DEFAULT 40
#define POT_IVRB_DEFAULT 40

// Externals defined in main.c.
extern TIM_HandleTypeDef htim1;
extern I2C_HandleTypeDef hi2c1;

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

  uint32_t Vout = (ADC_REF_VOLTAGE * adcValue) / ADC_MAX;  // Vout in mV
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
  uint32_t Vout = (ADC_REF_VOLTAGE * adcValue) / ADC_MAX;
  const uint32_t numerator = Vout * (47000 + 6200);
  const uint32_t denominator = 6200;
  uint16_t Vbat = (uint16_t)(numerator / denominator);
  memcpy(frame->data, &Vbat, sizeof(adcValue));
}

/* Voltage divider with R1 = 39kOhm and R2 = 15kOhm
 * Vbat = Vout * (R1 + R2) / R2
 * Vout is measured by ADC, Vout = ADC_REF_VOLTAGE * adcValue / ADC_MAX
 */
void ADCToVCCServoVoltageMessage(uint16_t adcValue, CANFrame *frame) {
  frame->id = VCC_SERVO_VOLTAGE_MESSAGE_ID;
  frame->dlc = VCC_SERVO_VOLTAGE_MESSAGE_DLC;
  uint32_t Vout = (ADC_REF_VOLTAGE * adcValue) / ADC_MAX;  // Vout in mV
  const uint32_t numerator = Vout * (39000 + 15000);
  const uint32_t denominator = 15000;
  uint16_t Vbat = (uint16_t)(numerator / denominator);  // Vbat in mV
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
  uint16_t Isense = (uint16_t)((ADC_REF_VOLTAGE * adcValue) / ADC_MAX);

  uint16_t current = Isense;
  if (current > maxCurrent) {
    current = maxCurrent;
  }
  memcpy(frame->data, &current, sizeof(current));
}

/* Set up potentiomenter terminals with default values.
 * TODO: split into two functions, one for servo and one for sensor.
 *       Make potentiometer values configurable over CAN as well.
 *
 * The TPL0102 potentiometer's end-to-end resistance is 100kOhm +/- 20%.
 * According to the schematic, VCC_Servo will have a maximum voltage output
 * of 10.80A, and a minimum of 2.47V - 3.11V depending on the actual maximum
 * resistance in the potentiometer (higher resistance => lower minimum).
 * The actual voltage will depend on the battery voltage, so make sure to
 * measure VCC_Servo with a multimeter before connecting a servo, to not burn
 * it. The corresponding range for VDD_Sensor is 5.10V maximum, and 3.22V-3.28V
 * minimum.
 *
 * The default value of POT_IVRA_DEFAULT is based on a servo with an operating
 * voltage of 6.0-8.4V.
 */
void InitPotentiometers(void) {
  uint8_t ivraWrite[2] = {POT_IVRA_ADDR, POT_IVRA_DEFAULT};
  uint8_t ivrbWrite[2] = {POT_IVRB_ADDR, POT_IVRB_DEFAULT};
  HAL_I2C_Master_Transmit(&hi2c1, POT_ADDR, ivraWrite, sizeof(ivraWrite),
                          portMAX_DELAY);
  HAL_I2C_Master_Transmit(&hi2c1, POT_ADDR, ivrbWrite, sizeof(ivrbWrite),
                          portMAX_DELAY);
}

/* Going towards the minimum pulse gives steering to the right. Going towards
 * maximum pulse gives steering to the left.
 */
void UpdatePWMDutyCycle(uint32_t *pulse, STEERING_DIRECTION *direction) {
  const uint32_t minPulse = htim1.Init.Period / 20;  // 5% duty cycle
  const uint32_t maxPulse = htim1.Init.Period / 10;  // 10% duty cycle
  const uint8_t pulseStep = 10;
  if (*pulse <= minPulse) {
    *pulse = minPulse;
    *direction = LEFT;
  } else if (*pulse >= maxPulse) {
    *pulse = maxPulse;
    *direction = RIGHT;
  }
  if (*direction == LEFT) {
    *pulse += pulseStep;
  } else {
    *pulse -= pulseStep;
  }
  // Write new pulse to CCR register
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, *pulse);
}
