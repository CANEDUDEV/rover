/******************************************************************************
 * @file app.h
 *
 * Contains functions that compose the servo application.
 *
 *****************************************************************************/
#ifndef APP_H
#define APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct {
  uint32_t id;
  uint8_t dlc;
  uint8_t data[8];  // NOLINT
} CANFrame;

typedef enum {
  LEFT = 0,
  RIGHT,
} STEERING_DIRECTION;

/*******************************************************
 * Convert ADC reading to a sensor power CAN message.
 *
 * @param adcValue The raw ADC reading.
 * @param frame CAN frame populated by the function.
 ******************************************************/
void ADCToSensorPowerMessage(uint16_t adcValue, CANFrame *frame);
void ADCToServoCurrentMessage(uint16_t adcValue, CANFrame *frame);
void ADCToBatteryVoltageMessage(uint16_t adcValue, CANFrame *frame);
void ADCToVCCServoVoltageMessage(uint16_t adcValue, CANFrame *frame);
void ADCToHBridgeWindingCurrentMessage(uint16_t adcValue, CANFrame *frame);
void InitPotentiometers(void);
void UpdatePWMDutyCycle(uint32_t *pulse, STEERING_DIRECTION *direction);

#ifdef __cplusplus
}
#endif

#endif /* APP_H */
