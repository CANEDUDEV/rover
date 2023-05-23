#include "app.h"

#include <string.h>

#include "ports.h"

#define ADC_REF_VOLTAGE 3300  // in mV
#define ADC_MAX 4095          // 12-bit ADC

#define POT_ADDR (0x53 << 1)  // Shift left to match STM32 specification
#define POT_IVRA_ADDR 0x0

// Cells that don't exist will report a value close to 0. We set a cell
// detection threshold voltage at 100 mV.
#define BATTERY_CELL_DETECTION_THRESHOLD 100

// CAN message lengths
#define BNS_CELLS_1_TO_4_DLC 8
#define BNS_CELLS_5_TO_6_DLC 4
#define BNS_REG_OUT_CURRENT_DLC 2
#define BNS_VBAT_OUT_CURRENT_DLC 4

static uint32_t VBatSenseRout;

void SetJumperConfig(JumperConfig jumperConfig) {
  switch (jumperConfig) {
    case ALL_OFF:
      VBatSenseRout = 10200;  // NOLINT
      break;
    case X11_ON:
      VBatSenseRout = 5100;  // NOLINT
      break;
    case X12_ON:
      VBatSenseRout = 3400;  // NOLINT
      break;
    case ALL_ON:
      VBatSenseRout = 2550;  // NOLINT
      break;
  }
}

void SetLEDColor(LED led, LEDColor color) {
  GPIO_TypeDef *redLEDPort = LED1_GPIO_Port;
  GPIO_TypeDef *greenLEDPort = LED2_GPIO_Port;
  uint16_t redLED = LED1_Pin;
  uint16_t greenLED = LED2_Pin;

  if (led == LED7) {
    redLEDPort = LED3_GPIO_Port;
    greenLEDPort = LED4_GPIO_Port;
    redLED = LED3_Pin;
    greenLED = LED4_Pin;
  }

  switch (color) {
    case NONE:
      HAL_GPIO_WritePin(redLEDPort, redLED, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(greenLEDPort, greenLED, GPIO_PIN_RESET);
      break;
    case RED:
      HAL_GPIO_WritePin(redLEDPort, redLED, GPIO_PIN_SET);
      HAL_GPIO_WritePin(greenLEDPort, greenLED, GPIO_PIN_RESET);
      break;
    case GREEN:
      HAL_GPIO_WritePin(redLEDPort, redLED, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(greenLEDPort, greenLED, GPIO_PIN_SET);
      break;
    case ORANGE:
      HAL_GPIO_WritePin(redLEDPort, redLED, GPIO_PIN_SET);
      HAL_GPIO_WritePin(greenLEDPort, greenLED, GPIO_PIN_SET);
      break;
  }
}

void BlinkLEDsRed(void) {
  static uint8_t blink = 0;
  if (blink > 0) {
    blink = 0;
    SetLEDColor(LED6, RED);
    SetLEDColor(LED7, RED);
  } else {
    blink = 1;
    SetLEDColor(LED6, NONE);
    SetLEDColor(LED7, NONE);
  }
}

void ConfigureVoltageRegulator(I2C_HandleTypeDef *hi2c, uint8_t potValue) {
  uint8_t ivraWrite[2] = {POT_IVRA_ADDR, potValue};
  HAL_I2C_Master_Transmit(hi2c, POT_ADDR, ivraWrite, sizeof(ivraWrite),
                          portMAX_DELAY);
}

/* Voltage divider with R1 = 1kOhm and R2 = 2kOhm
 * Vcell = Vout * (R1 + R2) / R2
 * Vout is measured by ADC, Vout = ADC_REF_VOLTAGE * adcValue / ADC_MAX
 */
uint16_t ADCToCellVoltage(const uint16_t adcValue) {
  uint32_t Vout = (ADC_REF_VOLTAGE * adcValue) / ADC_MAX;
  const uint32_t numerator = Vout * (1000 + 2000);
  const uint32_t denominator = 2000;
  uint16_t Vcell = (uint16_t)(numerator / denominator);
  return Vcell;
}

/* The LT6106 current sensor specifies that
 * Isense = Vout * Rin / (Rsense * Rout).
 * Vout is measured by ADC, Vout = ADC_REF_VOLTAGE * adcValue / ADC_MAX
 *
 * Resistances for Battery Node rev B:
 * Rin = 51 Ohm
 * Rsense = 0.005 Ohm
 * Rout = 5100 Ohm
 */
uint16_t ADCToRegOutCurrent(const uint16_t adcValue) {
  uint32_t Vout = (ADC_REF_VOLTAGE * adcValue) / ADC_MAX;
  const uint32_t Rin = 51;
  const uint32_t Rout = 5100;
  // Invert Rsense to avoid floating point, 1/0.005Ohm = 200 Ohm
  const uint32_t invRsense = 200;

  // Multiply by invRsense instead of dividing by Rsense
  uint16_t Isense = (uint16_t)((Vout * Rin * invRsense) / Rout);  // in mA
  return Isense;
}

/*
 * Uses LT6106 as well, with variable Rout configured by placing jumpers on X11
 * and X12.
 * Isense = Vout * Rin / (Rsense * Rout)
 *
 * Vout is measured by ADC, Vout = ADC_REF_VOLTAGE * adcValue / ADC_MAX
 *
 * Resistances:
 * Rin = 51 Ohm
 * Rsense = 0.5 mOhm
 * Rout: See VBatSenseRout
 *
 * Note: we use uint32_t because we can measure values from 20 mA to 120 A.
 */
uint32_t ADCToVbatOutCurrent(const uint16_t adcValue) {
  uint32_t Vout = (ADC_REF_VOLTAGE * adcValue) / ADC_MAX;  // in mV
  const uint32_t Rin = 51;
  // Invert Rsense to avoid floating point, 1/0.5mOhm = 2000 Ohm
  const uint32_t invRsense = 2000;

  // Multiply by invRsense instead of dividing by Rsense
  uint32_t Isense = (Vout * Rin * invRsense) / VBatSenseRout;  // in mA
  return Isense;
}

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers, readability-magic-numbers)
void ParseADCValues(const ADCReading *adcReading, BatteryNodeState *bns) {
  for (int i = 0; i < 4; i++) {
    bns->cells[i] = ADCToCellVoltage(adcReading->adc1Buf[i]);
  }
  bns->cells[4] = ADCToCellVoltage(adcReading->adc2Buf[0]);
  bns->cells[5] = ADCToCellVoltage(adcReading->adc2Buf[1]);
  bns->regOutCurrent = ADCToRegOutCurrent(adcReading->adc2Buf[2]);
  bns->vbatOutCurrent = ADCToVbatOutCurrent(adcReading->adc2Buf[3]);
}
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers, readability-magic-numbers)

BatteryCharge lowestCell(const BatteryCharge *cellCharge) {
  BatteryCharge lowest = CHARGE_100_PERCENT;
  for (int i = 0; i < BATTERY_CELLS_MAX; i++) {
    // Comparison based on enum ordering
    if (cellCharge[i] < lowest) {
      lowest = cellCharge[i];
    }
  }
  return lowest;
}

uint8_t SetChargeStateLED(const BatteryCharge *charge) {
  switch (*charge) {
    case CHARGE_100_PERCENT:
      SetLEDColor(LED6, GREEN);
      SetLEDColor(LED7, GREEN);
      return 0;
    case CHARGE_80_PERCENT:
      SetLEDColor(LED6, GREEN);
      SetLEDColor(LED7, NONE);
      return 0;
    case CHARGE_60_PERCENT:
      SetLEDColor(LED6, ORANGE);
      SetLEDColor(LED7, ORANGE);
      return 0;
    case CHARGE_40_PERCENT:
      SetLEDColor(LED6, ORANGE);
      SetLEDColor(LED7, NONE);
      return 0;
    case CHARGE_20_PERCENT:
      SetLEDColor(LED6, RED);
      SetLEDColor(LED7, RED);
      return 0;
    case LOW_VOLTAGE_CUTOFF:
    default:
      return 1;
  }
}

// Always report lowest detected charge to detect the most discharged cell.
BatteryCharge ReadBatteryCharge(const BatteryNodeState *bns) {
  BatteryCharge cellCharge[BATTERY_CELLS_MAX];
  for (int i = 0; i < BATTERY_CELLS_MAX; i++) {
    // If cell is not connected, we report it as fully charged to not use its
    // values in the low voltage detection logic.
    if (bns->cells[i] < BATTERY_CELL_DETECTION_THRESHOLD) {
      cellCharge[i] = CHARGE_100_PERCENT;
      continue;
    }

    if (bns->cells[i] <= LOW_VOLTAGE_CUTOFF) {
      cellCharge[i] = LOW_VOLTAGE_CUTOFF;
    } else if (bns->cells[i] <= CHARGE_20_PERCENT) {
      cellCharge[i] = CHARGE_20_PERCENT;
    } else if (bns->cells[i] <= CHARGE_40_PERCENT) {
      cellCharge[i] = CHARGE_40_PERCENT;
    } else if (bns->cells[i] <= CHARGE_60_PERCENT) {
      cellCharge[i] = CHARGE_60_PERCENT;
    } else if (bns->cells[i] <= CHARGE_80_PERCENT) {
      cellCharge[i] = CHARGE_80_PERCENT;
    } else {
      cellCharge[i] = CHARGE_100_PERCENT;
    }
  }
  return lowestCell(cellCharge);
}
