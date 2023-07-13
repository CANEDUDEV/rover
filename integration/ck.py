# CAN Kingdom letters for testing

from canlib import Frame

default_letter = Frame(id_=2031, data=[0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA])

# Give base number (0x400) and ask for response page 0
give_base_no = Frame(id_=0, dlc=8, data=[0, 1, 0, 0, 0, 4, 0, 0])

# Set communication mode to COMMUNICATE
communicate = Frame(id_=0, dlc=8, data=[0, 0, 0, 0x3, 0, 0, 0, 0])

# City IDs
battery_monitor_id = 1
servo_id = 2
motor_id = 3
sbus_receiver_id = 4
