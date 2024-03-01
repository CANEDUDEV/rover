#ifndef VOLTAGE_DIVIDER_H
#define VOLTAGE_DIVIDER_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  float r1;
  float r2;
} voltage_divider_t;

float reverse_voltage_division(float output_voltage,
                               const voltage_divider_t *divider);
#ifdef __cplusplus
}
#endif

#endif  // VOLTAGE_DIVIDER_H
