#ifndef LT6106_H
#define LT6106_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  float r_in;
  float r_out;
  float r_sense;
} lt6106_current_sensor_t;

float lt6106_sense_current(float output_voltage,
                           const lt6106_current_sensor_t *sensor);

#ifdef __cplusplus
}
#endif

#endif  // LT6106_H
