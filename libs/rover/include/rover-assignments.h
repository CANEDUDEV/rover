#ifndef ROVER_ASSIGNMENTS_H
#define ROVER_ASSIGNMENTS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>

typedef struct {
  uint16_t envelope;
  uint8_t city;
  uint8_t folder;
} rover_assignment_t;

typedef struct {
  rover_assignment_t* assignments;
  size_t assignment_count;
} rover_kingdom_t;

rover_kingdom_t* get_rover_kingdom(void);
void init_rover_kingdom(void);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* end of include guard: ROVER_ASSIGNMENTS_H */
