#include "float.h"

#include <stdio.h>
#include <string.h>

#include "test.h"

int main(void) {
  // NOLINTBEGIN(*-magic-numbers)
  const float inputs[] = {
      1.0F, -100.0F, 0.5F, -0.5F, 0.00123F, -0.00123F, 0.1666667F, 13.37F,
  };
  // NOLINTEND(*-magic-numbers)
  const char *expected[] = {
      "1.000000", "-100.000000", "0.500000", "-0.500000",
      "0.001230", "-0.001230",   "0.166666", "13.369999",
  };

  for (size_t i = 0; i < sizeof(inputs) / sizeof(float); i++) {
    char str[32];  // NOLINT(*-magic-numbers)
    float_sprint(str, inputs[i]);
    ASSERT(strcmp(str, expected[i]) == 0, "expected: %s, got: %s", expected[i],
           str);
  }
}
