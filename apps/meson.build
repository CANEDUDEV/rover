subdir('battery-monitor')
subdir('joystick')
subdir('sbus-receiver')
subdir('servo')

# Build binaries with bootloader. Loops through the app_binaries dictionary and
# creates binaries with both the bootloader and the app, using cat.
app_binaries = {
  'battery-monitor': [battery_monitor_bin],
  'joystick': [joystick_bin],
  'motor': [motor_bin], # Built from servo app
  'sbus-receiver': [sbus_receiver_bin],
  'servo': [servo_bin],
}

foreach name, app : app_binaries
  custom_target(
    '@0@-with-bootloader.bin'.format(name),
    build_by_default: true,
    capture: true, # This makes stdout of cat be captured and written to output file.
    command: [cat, '@INPUT@'],
    depends: [bootloader_bin, app],
    input: [bootloader_bin, app],
    output: ['@0@-with-bootloader.bin'.format(name)],
  )
endforeach

apps_tidy_files = [
  battery_monitor_tidy_files,
  joystick_tidy_files,
  sbus_receiver_tidy_files,
  servo_tidy_files,
]
