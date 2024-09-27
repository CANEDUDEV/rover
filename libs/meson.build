# Order is important
subdir('test-utils')
subdir('circuits')
subdir('arena')
subdir('float')
subdir('json')
subdir('can-kingdom')
subdir('rover')
subdir('stm32-common')
subdir('postmaster')

common_deps += [rover_dep, can_kingdom_mayor_dep, stm32_common_dep]

app_deps = [common_deps, circuits_dep, postmaster_dep]

common_libs_tidy_files = [
    can_kingdom_tidy_files,
    rover_tidy_files,
    stm32_common_tidy_files,
    circuits_tidy_files,
    arena_tidy_files,
    json_tidy_files,
    float_tidy_files,
    postmaster_tidy_files,
]
