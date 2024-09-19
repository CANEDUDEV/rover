#ifndef DEVICE_ID_H
#define DEVICE_ID_H

#include "ck-types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Gives a default rover-compatible ck_id_t given a city_address.
 */
ck_id_t get_default_ck_id(uint8_t city_address);

/**
 * ID will be cached after calls to read_ck_id().
 */
ck_id_t *get_cached_ck_id(void);

/**
 * Reads #ck_id from SPI flash.
 * Returns APP_OK on success, APP_NOT_OK or littlefs error codes otherwise.
 */
int read_ck_id(ck_id_t *ck_id);

/**
 * Writes given #ck_id to SPI flash.
 * Returns APP_OK on success, APP_NOT_OK or littlefs error codes otherwise.
 */
int write_ck_id(ck_id_t *ck_id);

/**
 * Returns a HW device unique serial number.
 */
uint32_t get_device_serial(void);

#ifdef __cplusplus
}
#endif

#endif  // DEVICE_ID_H
