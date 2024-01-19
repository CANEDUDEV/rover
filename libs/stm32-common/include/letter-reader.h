#ifndef LETTER_READER_H
#define LETTER_READER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "types.h"

typedef struct {
  uint32_t priority;
  int (*app_letter_handler_func)(const ck_folder_t *folder,
                                 const ck_letter_t *letter);

} letter_reader_cfg_t;

int init_letter_reader_task(letter_reader_cfg_t task);

#ifdef __cplusplus
}
#endif

#endif /* LETTER_READER_H*/
