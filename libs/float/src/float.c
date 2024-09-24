#include <stdio.h>

// The float functions from newlib-nano take up too much flash area to fit in
// the bootloader. We define our own dumb printer instead. Does not handle edge
// cases, but should be enough for our purposes.
int float_sprint(char *str, float num) {
  // Extract integer part
  int ipart = (int)num;

  // Extract floating part
  float fpart = num - (float)ipart;

  // 6 decimal places
  int fraction = (int)(fpart * 1e6);  // NOLINT(*magic-numbers)

  // Remove sign
  if (ipart < 0) {
    ipart = -ipart;
  }
  if (fraction < 0) {
    fraction = -fraction;
  }

  // Print to string
  int bytes = 0;
  // Extract sign
  if (num < 0) {
    bytes += sprintf(str, "-");
  }
  bytes += sprintf(str + bytes, "%d", ipart);
  bytes += sprintf(str + bytes, ".");
  bytes += sprintf(str + bytes, "%.6d", fraction);

  return bytes;
}

int float_print(float num) {
  char str[32];  // NOLINT(*magic-numbers)
  float_sprint(str, num);

  return printf("%s", str);
}
