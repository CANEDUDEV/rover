#ifndef POTENTIOMETER_H
#define POTENTIOMETER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

int init_potentiometer(void);
int configure_potentiometer(uint8_t pot_value);
int read_potentiometer_value(uint8_t *pot_value);

#ifdef __cplusplus
}
#endif

#endif /* POTENTIOMETER_H */
