#ifndef ARENA_H
#define ARENA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>

// Arena allocator where you BYOB (Bring Your Own Buffer)
typedef struct {
  char* memory;
  size_t capacity;
  size_t allocated;
} arena_t;

arena_t arena_init(char* mem, size_t capacity);
void* arena_alloc(arena_t* arena, size_t size);
void arena_reset(arena_t* arena);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* end of include guard: ARENA_H */
