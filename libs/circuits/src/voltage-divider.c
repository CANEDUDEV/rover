#include "voltage-divider.h"

float reverse_voltage_division(float output_voltage,
                               const voltage_divider_t *divider) {
  return output_voltage * (divider->r1 + divider->r2) / divider->r2;
}
