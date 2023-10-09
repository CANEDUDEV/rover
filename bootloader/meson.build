bootloader_inc = include_directories('include')

bootloader_src = files(
  'src' / 'ck-data.c',
  'src' / 'main.c',
)

app_linker_script = fs.copyfile(files('src' / 'app.ld'))

bootloader_linker_script = fs.copyfile(files('src' / 'bootloader.ld'))

bootloader_elf = executable(
  'bootloader.elf',
  [bootloader_src, common_libs_src],
  include_directories: [bootloader_inc, common_libs_inc],
  link_depends: bootloader_linker_script,
  link_args: ['-T@0@'.format(bootloader_linker_script.full_path())],
)

bootloader_bin = custom_target(
  output: 'bootloader.bin',
  input: bootloader_elf,
  command: [
    objcopy,
    '-S',
    '-O', 'binary',
    '--pad-to', '0x08010000', # Flash base + 64KB
    '--gap-fill', '0xFF', # Flash erase value
    '@INPUT@',
    '@OUTPUT@',
  ],
  depends: bootloader_elf,
  build_by_default: true,
)

bootloader_tidy_files = [bootloader_src]
