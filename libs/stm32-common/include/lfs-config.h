#ifndef LFS_CONFIG_H
#define LFS_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "lfs.h"

// Wrapper struct for littlefs config
typedef struct {
  const struct lfs_config *cfg;
  lfs_t *lfs;

} lfs_config_t;

lfs_config_t get_lfs_config(void);

int lfs_init(void);
int lfs_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* LFS_CONFIG_H */
