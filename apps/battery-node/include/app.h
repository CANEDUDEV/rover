/*******************************************************************************
 * @file app.h
 *
 * Provides data structures and application logic for the battery node
 * application.
 ******************************************************************************/
#ifndef APP_H
#define APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "stm32f3xx_hal.h"

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
void BlinkLEDsRed(void);

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

// Define some charge states in mV. Based on LiPo batteries. Assumes maximum
// cell voltage of 4200 mV and a low voltage cutoff at 3200 mV.
typedef enum {
  LOW_VOLTAGE_CUTOFF = 3200,
  CHARGE_20_PERCENT = 3400,
  CHARGE_40_PERCENT = 3600,
  CHARGE_60_PERCENT = 3800,
  CHARGE_80_PERCENT = 4000,
  CHARGE_100_PERCENT = 4200,
} BatteryCharge;

uint8_t SetChargeStateLED(const BatteryCharge *charge);

BatteryCharge ReadBatteryCharge(const BatteryNodeState *bns);

#ifdef __cplusplus
}
#endif

#endif /* APP_H */
