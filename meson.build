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
# Try to find python3.exe first, in case we're running in WSL.
python = find_program('python3.exe', 'python3', required: false)

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

generate_system_conf_script = files('rover_py' / 'generate-system-conf.py')

config_dir = custom_target(
    'config',
    output: 'config',
    command: [
        python,
        generate_system_conf_script,
        '--force',
        '--battery-monitor-bin',
        battery_monitor_bin,
        '--brake-bin',
        brake_bin,
        '--sbus-receiver-bin',
        sbus_receiver_bin,
        '--servo-bin',
        servo_bin,
        '-o',
        '@OUTPUT@',
    ],
)

release_files += fs.copyfile('rover.dbc')
release_script = fs.copyfile('scripts' / 'create-release.sh')
flasher_path = meson.project_source_root() / 'rover_py'
release_dir = 'release'

release_tgt = custom_target(
    'release',
    input: release_files,
    output: release_dir,
    command: [
        'bash',
        release_script,
        '--config-dir',
        config_dir,
        '--release-dir',
        release_dir,
        '--flasher-path',
        flasher_path,
        '@INPUT@',
    ],
    build_always_stale: true,
    build_by_default: true,
    depends: config_dir,
)

run_target(
    'flash',
    command: [
        python,
        release_dir / 'fw_update.py',
        'system',
        '--config',
        config_dir.full_path() / 'system.json',
        '--binary-dir',
        release_dir / 'binaries',
    ],
    depends: release_tgt,
)
