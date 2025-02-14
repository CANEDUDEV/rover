brake_inc = include_directories('include')

brake_src = files(
    'src' / 'ck.c',
    'src' / 'ck-data.c',
    'src' / 'interrupts.c',
    'src' / 'main.c',
    'src' / 'peripherals.c',
    'src' / 'potentiometer.c',
    'src' / 'report.c',
    'src' / 'wheel-speed.c',
)

brake_elf = executable(
    'brake.elf',
    brake_src,
    include_directories: brake_inc,
    dependencies: app_deps,
    link_depends: app_linker_script,
    link_args: ['-T@0@'.format(app_linker_script.full_path())],
)

brake_bin = custom_target(
    output: 'brake.bin',
    input: brake_elf,
    command: [objcopy, '-O', 'binary', '-S', '@INPUT@', '@OUTPUT@'],
    build_by_default: true,
)

run_target(
    'swd-flash-brake',
    command: [
        openocd,
        '-f',
        openocd_config,
        '-c',
        'program @0@ verify'.format(bootloader_elf.full_path()),
        '-c',
        'program @0@ verify reset exit'.format(brake_elf.full_path()),
    ],
    depends: [bootloader_elf, brake_elf],  # required because full_path doesn't create automatic dependency on these objects
)

release_files += [brake_elf, brake_bin]

brake_tidy_files = brake_src
