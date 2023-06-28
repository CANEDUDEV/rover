#ifndef FREERTOS_TASKS_H
#define FREERTOS_TASKS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct {
  uint16_t battery_monitor_period_ms;
  uint16_t battery_report_period_ms;
} task_periods_t;

void task_init(void);
void set_task_periods(task_periods_t *task_periods);

#ifdef __cplusplus
}
#endif

#endif /* FREERTOS_TASKS_H */
