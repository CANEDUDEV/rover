#include "device-id.h"

#include <stdio.h>
#include <string.h>

#include "common-peripherals.h"
#include "error.h"
#include "json.h"
#include "jsondb.h"
#include "rover-defs.h"
#include "stm32f3xx_ll_utils.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"

static ck_id_t cached_ck_id;

ck_id_t get_default_ck_id(uint8_t city_address) {
  ck_id_t ck_id = {
      .city_address = city_address,
      .base_no = ROVER_BASE_NUMBER,
      .base_no_is_known = true,
      .base_no_has_extended_id = false,
  };
  memset(ck_id.group_addresses, 0, sizeof(ck_id.group_addresses));
  return ck_id;
}

ck_id_t *get_cached_ck_id(void) {
  return &cached_ck_id;
}

int read_ck_id(ck_id_t *ck_id) {
  json_object_t *json = get_jsondb();
  if (!json) {
    printf("error: JSON DB not initialized or does not exist.\r\n");
    return APP_NOT_OK;
  }

  json_object_t *ck_id_json = json_get_object("ck_id", json);
  if (!ck_id_json) {
    printf("error: no CK ID stored.\r\n");
    return APP_NOT_OK;
  }

  // Ignore group addresses for now.
  json_object_t *city_address = json_get_object("city_address", ck_id_json);
  json_object_t *base_no = json_get_object("base_no", ck_id_json);
  json_object_t *base_no_has_extended_id =
      json_get_object("base_no_has_extended_id", ck_id_json);
  json_object_t *base_no_is_known =
      json_get_object("base_no_is_known", ck_id_json);

  if (!(city_address && base_no && base_no_has_extended_id &&
        base_no_is_known)) {
    printf("error: ck_id missing required fields\r\n");
    return APP_NOT_OK;
  }

  ck_id_t id = {
      .city_address = city_address->value->int_,
      .base_no = base_no->value->int_,
      .base_no_has_extended_id = base_no_has_extended_id->value->boolean,
      .base_no_is_known = base_no_is_known->value->boolean,
  };

  if (ck_check_ck_id(&id) != CK_OK) {
    printf("error: invalid ck_id stored\r\n");
    return APP_NOT_OK;
  }

  cached_ck_id = id;
  memcpy(ck_id, &id, sizeof(ck_id_t));

  return APP_OK;
}

int write_ck_id(ck_id_t *ck_id) {
  json_object_t *json = get_jsondb();
  if (!json) {
    printf("error: JSON DB not initialized or does not exist.\r\n");
    return APP_NOT_OK;
  }
  // NOLINTBEGIN(*-magic-numbers)
  char str[128];
  char base_no_has_extended_id[8];
  char base_no_is_known[8];
  // NOLINTEND(*-magic-numbers)
  if (ck_id->base_no_is_known) {
    strcpy(base_no_is_known, "true");
  } else {
    strcpy(base_no_is_known, "false");
  }
  if (ck_id->base_no_has_extended_id) {
    strcpy(base_no_has_extended_id, "true");
  } else {
    strcpy(base_no_has_extended_id, "false");
  }

  sprintf(str,
          "\"ck_id\":{\"city_address\":%u,\"base_no\":%lu,"
          "\"base_no_has_extended_id\":%s,\"base_no_is_known\":%s}",
          ck_id->city_address, (unsigned long)ck_id->base_no,
          base_no_has_extended_id, base_no_is_known);

  if (json_insert_object(str, json) != APP_OK) {
    printf("error: couldn't insert: %s\r\n", str);
    return APP_NOT_OK;
  }

  if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
    return jsondb_update_async(json);
  }

  return jsondb_update(json);
}

uint32_t get_device_serial(void) {
  uint32_t uid[3] = {
      LL_GetUID_Word0(),
      LL_GetUID_Word1(),
      LL_GetUID_Word2(),
  };

  common_peripherals_t *common_peripherals = get_common_peripherals();
  return HAL_CRC_Calculate(&common_peripherals->hcrc, uid,
                           sizeof(uid) / sizeof(uint32_t));
}
