bootloader_inc = include_directories('include')

bootloader_src = files(
    'src' / 'ck-data.c',
    'src' / 'main.c',
    'src' / 'postmaster.c',
)

app_linker_script = fs.copyfile(files('src' / 'app.ld'))

bootloader_linker_script = fs.copyfile(files('src' / 'bootloader.ld'))

bootloader_elf = executable(
    'bootloader.elf',
    bootloader_src,
    c_args: ['-Os'],  # Optimize for size
    include_directories: bootloader_inc,
    dependencies: [common_deps, json_dep],
    link_depends: bootloader_linker_script,
    link_args: ['-T@0@'.format(bootloader_linker_script.full_path())],
)

bootloader_bin = custom_target(
    output: 'bootloader.bin',
    input: bootloader_elf,
    command: [
        objcopy,
        '-S',
        '-O',
        'binary',
        '--pad-to',
        '0x08010000',  # Flash base + 64KB
        '--gap-fill',
        '0xFF',  # Flash erase value
        '@INPUT@',
        '@OUTPUT@',
    ],
    depends: bootloader_elf,
    build_by_default: true,
)

run_target(
    'swd-flash-bootloader',
    command: [
        openocd,
        '-f',
        openocd_config,
        '-c',
        'program @0@ verify reset exit'.format(bootloader_elf.full_path()),
    ],
)

release_files += [bootloader_elf, bootloader_bin]

bootloader_tidy_files = [bootloader_src]
