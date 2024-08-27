#include "arena.h"

#include <string.h>

// NOLINTNEXTLINE(readability-non-const-parameter)
arena_t arena_init(char* mem, size_t capacity) {
  arena_t arena = {
      .memory = mem,
      .capacity = capacity,
      .allocated = 0,
  };

  arena_reset(&arena);

  return arena;
}

void* arena_alloc(arena_t* arena, size_t size) {
  if (arena->allocated + size > arena->capacity) {
    return NULL;
  }

  char* ptr = arena->memory + arena->allocated;
  arena->allocated += size;
  return ptr;
}

void arena_reset(arena_t* arena) {
  arena->allocated = 0;
  memset(arena->memory, 0, arena->capacity);
}
