
#include "potentiometer.h"

#include "peripherals.h"

#define POT_ADDR (0x50 << 1)  // Shift left to match STM32 specification
#define POT_IVRA_ADDR 0x0     // VCC_Servo potentiometer i2c address
#define POT_IVRB_ADDR 0x1     // VDD_Sensor potentiometer i2c address

void configure_servo_potentiometer(uint8_t pot_value) {
  peripherals_t *peripherals = get_peripherals();
  uint8_t ivra_write[2] = {POT_IVRA_ADDR, pot_value};
  HAL_I2C_Master_Transmit(&peripherals->hi2c1, POT_ADDR, ivra_write,
                          sizeof(ivra_write), HAL_MAX_DELAY);
}

void configure_sensor_potentiometer(uint8_t pot_value) {
  peripherals_t *peripherals = get_peripherals();
  uint8_t ivrb_write[2] = {POT_IVRB_ADDR, pot_value};
  HAL_I2C_Master_Transmit(&peripherals->hi2c1, POT_ADDR, ivrb_write,
                          sizeof(ivrb_write), HAL_MAX_DELAY);
}

void configure_both_potentiometers(uint8_t servo_pot, uint8_t sensor_pot) {
  peripherals_t *peripherals = get_peripherals();
  uint8_t ivr_write[3] = {POT_IVRA_ADDR, servo_pot, sensor_pot};
  HAL_I2C_Master_Transmit(&peripherals->hi2c1, POT_ADDR, ivr_write,
                          sizeof(ivr_write), HAL_MAX_DELAY);
}
