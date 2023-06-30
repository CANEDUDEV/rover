# Servo board testing frames

from canlib import Frame

# Assign envelope 0x200 to folder 2
assign_folder2 = Frame(id_=0, dlc=8, data=[0, 2, 0, 2, 0, 0, 2, 0x3])

# Assign envelope 0x201 to folder 3
assign_folder3 = Frame(id_=0, dlc=8, data=[0, 2, 1, 2, 0, 0, 3, 0x3])

# Assign envelope 0x202 to folder 4
assign_folder4 = Frame(id_=0, dlc=8, data=[0, 2, 2, 2, 0, 0, 4, 0x3])

# Assign envelope 0x203 to folder 5
assign_folder5 = Frame(id_=0, dlc=8, data=[0, 2, 3, 2, 0, 0, 5, 0x3])

# Assign envelope 0x204 to folder 6
assign_folder6 = Frame(id_=0, dlc=8, data=[0, 2, 4, 2, 0, 0, 6, 0x3])

# Assign envelope 0x205 to folder 7
assign_folder7 = Frame(id_=0, dlc=8, data=[0, 2, 5, 2, 0, 0, 7, 0x3])

# Assign envelope 0x206 to folder 8
assign_folder8 = Frame(id_=0, dlc=8, data=[0, 2, 6, 2, 0, 0, 8, 0x3])

# Assign envelope 0x207 to folder 9
assign_folder9 = Frame(id_=0, dlc=8, data=[0, 2, 7, 2, 0, 0, 9, 0x3])

# Assign envelope 0x208 to folder 10
assign_folder10 = Frame(id_=0, dlc=8, data=[0, 2, 8, 2, 0, 0, 10, 0x3])

# Set potentiometer value to 35
set_potentiometer_35 = Frame(id_=0x205, dlc=2, data=[0x23, 0])

# Set PWM frequency to 333 Hz
set_pwm_conf_333hz = Frame(id_=0x206, dlc=2, data=[0x4D, 0x1])

# Set steering PWM pulse to 1000 µs
steer_pulse_1000 = Frame(id_=0x207, dlc=3, data=[0, 0xE8, 0x3])

# Set steering PWM pulse to 2000 µs
steer_pulse_2000 = Frame(id_=0x207, dlc=3, data=[0, 0xD0, 0x7])

# Set steering angle to 90 degrees
steer_angle_90 = Frame(id_=0x207, dlc=3, data=[1, 0x5A, 0])

# Set steering angle to -90 degrees
steer_angle_minus_90 = Frame(id_=0x207, dlc=3, data=[1, 0xA6, 0xFF])

# Set measure frequency to 500 ms. Ignore report frequency.
set_measure_freq_500ms = Frame(id_=0x208, dlc=4, data=[0xF4, 0x1, 0, 0])

# Set report frequency to 1000 ms. Ignore measure frequency.
set_report_freq_1000ms = Frame(id_=0x208, dlc=4, data=[0, 0, 0xE8, 0x3])
