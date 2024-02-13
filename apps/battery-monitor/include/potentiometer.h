#ifndef POTENTIOMETER_H
#define POTENTIOMETER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

int configure_potentiometer(uint8_t pot_value);
uint8_t get_potentiometer_value(void);

#ifdef __cplusplus
}
#endif

#endif /* POTENTIOMETER_H */
