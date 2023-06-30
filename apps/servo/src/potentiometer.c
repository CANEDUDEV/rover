
#include "potentiometer.h"

#include "peripherals.h"

#define POT_ADDR (0x50 << 1)  // Shift left to match STM32 specification
#define POT_IVRA_ADDR 0x0     // VCC_Servo potentiometer i2c address
#define POT_IVRB_ADDR 0x1     // VDD_Sensor potentiometer i2c address

void configure_servo_potentiometer(uint8_t pot_value) {
  peripherals_t *peripherals = get_peripherals();
  uint8_t ivraWrite[2] = {POT_IVRA_ADDR, pot_value};
  HAL_I2C_Master_Transmit(&peripherals->hi2c1, POT_ADDR, ivraWrite,
                          sizeof(ivraWrite), HAL_MAX_DELAY);
}

void configure_sensor_potentiometer(uint8_t pot_value) {
  peripherals_t *peripherals = get_peripherals();
  uint8_t ivrbWrite[2] = {POT_IVRB_ADDR, pot_value};
  HAL_I2C_Master_Transmit(&peripherals->hi2c1, POT_ADDR, ivrbWrite,
                          sizeof(ivrbWrite), HAL_MAX_DELAY);
}
