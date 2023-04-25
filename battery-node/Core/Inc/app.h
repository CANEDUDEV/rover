#ifndef APP_H
#define APP_H

#include <stdint.h>

#include "stm32f3xx_hal.h"
#include "utils.h"

#define ADC1_NUM_CHANNELS 4
#define ADC2_NUM_CHANNELS 4

#define BATTERY_CELLS_MAX 6  // Max number of battery cells supported

#define POT_IVRA_DEFAULT 40  // Default potentiometer value

void ConfigureVoltageRegulator(I2C_HandleTypeDef *hi2c, uint8_t potValue);

/*
 * X11 and X12 jumper configuration
 * | X11 | X12 | Rout      |
 * |-----|-----|-----------|
 * | OFF | OFF | 10.2 kOhm |
 * | ON  | OFF | 5.1 kOhm  |
 * | OFF | ON  | 3.4 kOhm  |
 * | ON  | ON  | 2.55 kOhm |
 */
typedef enum {
  ALL_OFF,
  X11_ON,
  X12_ON,
  ALL_ON,
} JumperConfig;

void SetJumperConfig(JumperConfig jumperConfig);

typedef enum {
  NONE = 0,
  RED,
  GREEN,
  ORANGE,
} LEDColor;

// Naming based on battery node schematic
typedef enum {
  LED6 = 0,
  LED7,
} LED;

void SetLEDColor(LED led, LEDColor color);

typedef struct {
  uint16_t adc1Buf[ADC1_NUM_CHANNELS];
  uint16_t adc2Buf[ADC2_NUM_CHANNELS];
} ADCReading;

typedef struct {
  uint16_t cells[BATTERY_CELLS_MAX];
  uint16_t regOutCurrent;
  uint32_t vbatOutCurrent;
} BatteryNodeState;

// ParseADCValues parses an ADCReading and populates a BatteryNodeState struct.
void ParseADCValues(const ADCReading *adcReading, BatteryNodeState *bns);

typedef struct {
  CANFrame cells1To4;
  CANFrame cells5To6;
  CANFrame regOutCurrent;
  CANFrame vbatOutCurrent;
} BatteryNodeStateMessage;

// Only the DLC and the Data of the BatteryNodeStateMessage members are
// populated. The user needs to provide CAN IDs themselves.
void PopulateBNSMessage(const BatteryNodeState *bns,
                        BatteryNodeStateMessage *bnsMsg);

#endif /* APP_H */
