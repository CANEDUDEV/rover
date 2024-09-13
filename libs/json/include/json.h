#ifndef JSON_H
#define JSON_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

#ifndef JSON_MAX_SIZE
#define JSON_MAX_SIZE 4096
#endif

enum json_value_type {
  JSON_INT,
  JSON_FLOAT,
  JSON_STRING,
  JSON_BOOL,
  JSON_NULL,
  JSON_ARRAY,
  JSON_OBJECT,
};

typedef union {
  char *string;
  int int_;
  float float_;
  bool boolean;
} json_value_t;

typedef struct json_object {
  char *name;
  enum json_value_type type;
  json_value_t *value;

  struct json_object *prev;
  struct json_object *next;
  struct json_object *child;

} json_object_t;

json_object_t *json_parse(const char *data);
json_object_t *json_get_object(const char *name, json_object_t *root);
int json_insert_object(const char *json, json_object_t *root);

void json_sprint(json_object_t *root, char *str);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* end of include guard: JSON_H */
