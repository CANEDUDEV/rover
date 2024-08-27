arena_inc = include_directories('include')

arena_src = files('src' / 'arena.c')

arena_lib = library('arena', arena_src, include_directories: arena_inc)

arena_dep = declare_dependency(
    include_directories: arena_inc,
    link_with: arena_lib,
)

subdir('tests')

arena_tidy_files = [arena_src, arena_test_tidy_files]
