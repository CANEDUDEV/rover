#ifndef SPI_FLASH_H
#define SPI_FLASH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "lfs.h"

const struct lfs_config *get_spi_flash_lfs_config(void);

#ifdef __cplusplus
}
#endif

#endif /* SPI_FLASH_H */
