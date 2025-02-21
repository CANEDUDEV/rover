sbus_receiver_inc = include_directories('include')

sbus_receiver_src = files(
    'src' / 'ck-data.c',
    'src' / 'freertos-tasks.c',
    'src' / 'interrupts.c',
    'src' / 'main.c',
    'src' / 'peripherals.c',
    'src' / 'sbus.c',
    'src' / 'steering.c',
)

sbus_receiver_elf = executable(
    'sbus-receiver.elf',
    sbus_receiver_src,
    include_directories: sbus_receiver_inc,
    dependencies: [app_deps, float_dep],
    link_depends: app_linker_script,
    link_args: ['-T@0@'.format(app_linker_script.full_path())],
)

sbus_receiver_bin = custom_target(
    output: 'sbus-receiver.bin',
    input: sbus_receiver_elf,
    command: [objcopy, '-O', 'binary', '-S', '@INPUT@', '@OUTPUT@'],
    build_by_default: true,
)

run_target(
    'swd-flash-sbus-receiver',
    command: [
        openocd,
        '-f',
        openocd_config,
        '-c',
        'program @0@ verify'.format(bootloader_elf.full_path()),
        '-c',
        'program @0@ verify reset exit'.format(sbus_receiver_elf.full_path()),
    ],
    depends: [bootloader_elf, sbus_receiver_elf],  # required because full_path doesn't create automatic dependency on these objects
)

release_files += [sbus_receiver_elf, sbus_receiver_bin]

sbus_receiver_tidy_files = sbus_receiver_src
