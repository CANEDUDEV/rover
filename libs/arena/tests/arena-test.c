#include "arena.h"

#include <stdio.h>
#include <string.h>

#include "test.h"

void test_arena_reset(void);
void test_arena_alloc(void);

const size_t capacity = 128;

int main(void) {
  test_arena_reset();
  test_arena_alloc();
}

void test_arena_reset(void) {
  char zero_buffer[capacity];
  memset(zero_buffer, 0, capacity);

  char buffer[capacity];
  arena_t arena = arena_init(buffer, capacity);

  ASSERT(memcmp(arena.memory, &zero_buffer, capacity) == 0,
         "arena not initialized properly.");

  int* num = arena_alloc(&arena, sizeof(int));
  *num = 123;  // NOLINT
  ASSERT(memcmp(arena.memory, &zero_buffer, capacity) != 0,
         "memory should have been written.");

  arena_reset(&arena);

  ASSERT(memcmp(arena.memory, &zero_buffer, capacity) == 0,
         "arena not reset properly.");

  ASSERT(arena.allocated == 0, "expected: %zu, got: %zu", 0L, arena.allocated);
}

void test_arena_alloc(void) {
  char buffer[capacity];
  arena_t arena = arena_init(buffer, capacity);

  for (size_t i = 0; i < 4; i++) {
    ASSERT(arena_alloc(&arena, capacity / 4) != NULL, "alloc %zu was NULL.", i);
  }

  ASSERT(arena_alloc(&arena, 1) == NULL,
         "expected alloc when full to return NULL");
}
