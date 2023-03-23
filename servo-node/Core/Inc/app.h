#ifndef APP_H
#define APP_H

#include <stdint.h>

#define CAN_MAX_DLC 8

typedef struct {
  uint32_t id;
  uint8_t dlc;
  uint8_t data[CAN_MAX_DLC];
} CANFrame;

typedef enum {
  LEFT = 0,
  RIGHT,
} STEERING_DIRECTION;

void ADCToSensorPowerMessage(uint16_t adcValue, CANFrame *frame);
void ADCToServoCurrentMessage(uint16_t adcValue, CANFrame *frame);
void ADCToBatteryVoltageMessage(uint16_t adcValue, CANFrame *frame);
void ADCToVCCServoVoltageMessage(uint16_t adcValue, CANFrame *frame);
void ADCToHBridgeWindingCurrentMessage(uint16_t adcValue, CANFrame *frame);
void InitPotentiometers(void);
void UpdatePWMDutyCycle(uint32_t *pulse, STEERING_DIRECTION *direction);

#endif /* APP_H */
