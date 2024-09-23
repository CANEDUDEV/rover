#ifndef LFS_WRAPPER_H
#define LFS_WRAPPER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>

// Parameters name and data should stay in scope during run-time if using
// non-blocking writes.
typedef struct {
  const char *name;
  void *data;
  size_t size;
} file_t;

void lfs_init(void);
void lfs_deinit(void);

int init_lfs_task(uint32_t write_priority);

// Blocking write
int write_file(const file_t *file);

// Asynchronous write. Blocks until item is queued.
// If this takes too long, increase LFS_QUEUE_LENGTH.
int write_file_async(const file_t *file);

// Blocking read.
int read_file(file_t *file);

int format_and_mount(void);

#ifdef __cplusplus
}
#endif

#endif /* LFS_WRAPPER_H */
