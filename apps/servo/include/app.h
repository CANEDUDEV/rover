/******************************************************************************
 * @file app.h
 *
 * Contains functions that compose the servo application.
 *
 *****************************************************************************/
#ifndef APP_H
#define APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum {
  LEFT = 0,
  RIGHT,
} STEERING_DIRECTION;

void InitPotentiometers(void);
void UpdatePWMDutyCycle(uint32_t *pulse, STEERING_DIRECTION *direction);

#ifdef __cplusplus
}
#endif

#endif /* APP_H */
