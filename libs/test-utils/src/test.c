#include "test.h"

bool is_acceptable_measurement(measurement_t *measurement) {
  float min_accepted_measurement =
      measurement->expected_value - measurement->accepted_error;
  float max_accepted_measurement =
      measurement->expected_value + measurement->accepted_error;

  return (measurement->actual_value >= min_accepted_measurement &&
          measurement->actual_value <= max_accepted_measurement);
}
