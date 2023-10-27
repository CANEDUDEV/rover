#ifndef CK_RX_LETTERS_H
#define CK_RX_LETTERS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "types.h"

int process_set_servo_voltage_letter(const ck_letter_t *letter);
int process_pwm_conf_letter(const ck_letter_t *letter);
int process_steering_letter(const ck_letter_t *letter);
int process_steering_trim_letter(const ck_letter_t *letter);
int process_report_freq_letter(const ck_letter_t *letter);
int process_reverse_letter(const ck_letter_t *letter);

#ifdef __cplusplus
}
#endif

#endif /* CK_RX_LETTERS_H */
