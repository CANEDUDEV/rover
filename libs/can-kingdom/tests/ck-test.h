#ifndef CK_TEST_H
#define CK_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ck-types.h"

// Some helpers for testing
const uint64_t test_ean_no = 123;
const uint64_t test_serial_no = 123;
const uint8_t test_city_address = 123;
const uint32_t test_base_no = 123;
const uint32_t illegal_can_std_id = CK_CAN_MAX_STD_ID + 1;
const uint32_t illegal_can_ext_id = CK_CAN_MAX_EXT_ID + 1;
const uint64_t illegal_ean_no = CK_MAX_EAN_NO + 1;

#ifdef __cplusplus
}
#endif

#endif /* CK_TEST_H */
