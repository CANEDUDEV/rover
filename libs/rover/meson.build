rover_inc = include_directories('include')

rover_helpers_src = files('src' / 'rover-helpers.c')

rover_src = [files('src' / 'rover.c'), rover_helpers_src]

rover_dep = declare_dependency(
    include_directories: rover_inc,
    sources: rover_src,
)

rover_king_dep = [rover_dep, can_kingdom_king_dep]

subdir('tests')

rover_tidy_files = [rover_src, rover_test_tidy_files]
