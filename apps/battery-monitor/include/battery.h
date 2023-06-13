#ifndef BATTERY_H
#define BATTERY_H

#ifdef __cplusplus
extern "C" {
#endif

// Cells that don't exist will report a value close to 0. We set a cell
// detection threshold voltage at 100 mV.
#define BATTERY_CELL_DETECTION_THRESHOLD 100

#ifdef __cplusplus
}
#endif

#endif /* BATTERY_H */
