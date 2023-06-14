# CAN Kingdom letters for testing

from canlib import Frame

default_letter = Frame(id_=2031, data=[0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA])

give_base_no = Frame(
    id_=0, dlc=8, data=[0, 1, 0, 0, 0, 2, 0, 0]
)  # Give base number and ask for response page 0

communicate = Frame(
    id_=0, dlc=8, data=[0, 0, 0, 0x3, 0, 0, 0, 0]
)  # Set communication mode to COMMUNICATE
