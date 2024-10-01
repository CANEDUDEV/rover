#include <stdio.h>

#include "json.h"
#include "jsondb.h"
#include "servo.h"

void settings_init(void) {
  json_object_t *json = get_jsondb();
  json_object_t *settings = json_get_object("settings", json);
  if (!settings) {
    printf("note: no settings stored\r\n");
    return;
  }

  json_object_t *reverse = json_get_object("reverse", settings);
  if (!reverse) {
    return;
  }

  if (reverse->type != JSON_BOOL) {
    printf("note: invalid reverse setting, using default\r\n");
  }

  servo_state_t *s = get_servo_state();
  s->reverse = reverse->value->boolean;
}
