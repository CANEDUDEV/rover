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
    dependencies: common_deps,
    link_depends: app_linker_script,
    link_args: ['-T@0@'.format(app_linker_script.full_path())],
)

brake_bin = custom_target(
    output: 'brake.bin',
    input: brake_elf,
    command: [objcopy, '-O', 'binary', '-S', '@INPUT@', '@OUTPUT@'],
    depends: brake_elf,
    build_by_default: true,
)

brake_tidy_files = brake_src
