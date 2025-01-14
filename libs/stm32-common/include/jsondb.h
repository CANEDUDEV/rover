#ifndef JSONDB_H
#define JSONDB_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>

#include "json.h"

json_object_t *get_jsondb(void);
char *get_jsondb_raw(void);
size_t get_jsondb_max_size(void);

void jsondb_init(void);
int jsondb_update(json_object_t *new_json);
int jsondb_update_async(json_object_t *new_json);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* end of include guard: JSONDB_H */
