#include <stdio.h>

#include "error.h"
#include "json.h"
#include "lfs-wrapper.h"

static char db_storage[JSON_MAX_SIZE];

static file_t db_file = {
    .name = "settings.json",
    .data = db_storage,
    .size = sizeof(db_storage),
};

static json_object_t *jsondb = NULL;

static void jsondb_sync(json_object_t *new_json);

json_object_t *get_jsondb(void) {
  return jsondb;
}

char *get_jsondb_raw(void) {
  return db_storage;
}

size_t get_jsondb_max_size(void) {
  return sizeof(db_storage);
}

void jsondb_init(void) {
  if (read_file(&db_file) != APP_OK) {
    printf("note: creating new JSON DB\r\n");
    sprintf(db_file.data, "{}");
  }

  jsondb = json_parse(db_file.data);
  if (!jsondb) {
    printf("error: corrupt JSON DB, creating new JSON DB.\r\n");
    sprintf(db_file.data, "{}");
    jsondb = json_parse(db_file.data);
  }
}

int jsondb_update(json_object_t *new_json) {
  jsondb_sync(new_json);

  if (write_file(&db_file) != APP_OK) {
    printf("error: failed to sync JSON DB\r\n");
    return APP_NOT_OK;
  }

  return APP_OK;
}

int jsondb_update_async(json_object_t *new_json) {
  jsondb_sync(new_json);

  if (write_file_async(&db_file) != APP_OK) {
    printf("error: failed to enqueue JSON DB sync\r\n");
    return APP_NOT_OK;
  }

  return APP_OK;
}

static void jsondb_sync(json_object_t *new_json) {
  json_object_t *target_json = jsondb;
  if (new_json) {
    target_json = new_json;
  }

  json_sprint(db_file.data, target_json);
}
