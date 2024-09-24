project(
    'Rover',
    'c',
    meson_version: '>=1.2.0',
    version: '0.1',
    default_options: [
        'optimization=2',
        'default_library=static',
        'warning_level=3',
        'werror=true',
        'c_std=c17',
    ],
)

add_project_arguments('-DLFS_NO_MALLOC', language: 'c')
add_project_arguments('-O0', '-g', language: 'c', native: true)

cc_native = meson.get_compiler('c', native: true)
math_dep = cc_native.find_library('m')

objcopy = find_program('objcopy')
clang_tidy = find_program('clang-tidy')
cat = find_program('cat')

stm32f3_hal_dep = dependency('stm32f3-hal')
freertos_dep = dependency('freertos')
littlefs_dep = dependency('littlefs')
fff_dep = dependency('fff')

common_deps = [stm32f3_hal_dep, freertos_dep, littlefs_dep]

# Populated in subdirs
release_files = []

# Order is important
fs = import('fs')
subdir('libs')
subdir('bootloader')
subdir('apps')
subdir('docs')

tidy_files = [apps_tidy_files, bootloader_tidy_files, common_libs_tidy_files]

# Override default meson clang-tidy target
run_target(
    'clang-tidy',
    command: [
        clang_tidy,
        '--config-file',
        meson.project_source_root() / '.clang-tidy',
        '--quiet',
        '-p',
        meson.project_build_root(),
        tidy_files,
    ],
)

run_target(
    'check',
    command: [
        'bash',
        meson.project_source_root() / 'scripts' / 'check-src.sh',
        meson.project_source_root(),
        meson.project_build_root(),
    ],
)

release_files += fs.copyfile('rover.dbc')
release_files += fs.copyfile('system.json')
release_script = fs.copyfile('scripts' / 'create-release.sh')

custom_target(
    'release',
    input: release_files,
    output: 'release',
    command: [
        'bash',
        release_script,
        '--flasher-path',
        meson.project_source_root() / 'rover_py',
        '@INPUT@',
    ],
    build_always_stale: true,
    build_by_default: true,
)
