from time import sleep
from canlib import canlib, Frame

default_letter = Frame(id_=2031, data=[0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA])
give_base_no = Frame(id_=0, dlc=8, data=[0,1,0,0,0,2,0,0]) # Give base number and ask for response page 0
assign_folder2 = Frame(id_=0, dlc=8, data=[0,2,0,1,0,0,2,0x3]) # Assign envelope 0x100 to folder 2
assign_folder3 = Frame(id_=0, dlc=8, data=[0,2,1,1,0,0,3,0x3]) # Assign envelope 0x101 to folder 3
assign_folder4 = Frame(id_=0, dlc=8, data=[0,2,2,1,0,0,4,0x3]) # Assign envelope 0x102 to folder 4
communicate = Frame(id_=0, dlc=8, data=[0,0,0,0x3,0,0,0,0]) # Set communication mode to COMMUNICATE
change_bitrate = Frame(id_=0, dlc=8, data=[0,8,0,0,9,8,1,1]) # Set bitrate to 500kbit/s
reset = Frame(id_=0, dlc=8, data=[0,0,0,0x55,0,0,0,0]) # Reset communication and set communication mode to SILENT

with canlib.openChannel(channel=0, flags=canlib.Open.REQUIRE_INIT_ACCESS, bitrate=canlib.Bitrate.BITRATE_125K) as ch:
    ch.setBusOutputControl(canlib.Driver.NORMAL)
    ch.busOn()

    ch.writeWait(default_letter, -1)
    ch.writeWait(give_base_no, -1)
    ch.writeWait(assign_folder2, -1)
    ch.writeWait(assign_folder3, -1)
    ch.writeWait(assign_folder4, -1)
    ch.writeWait(communicate, -1)
    sleep(0.5) # give time for communication
    ch.writeWait(change_bitrate, -1)
    ch.writeWait(reset, -1)

    ch.busOff()

with canlib.openChannel(channel=0, flags=canlib.Open.REQUIRE_INIT_ACCESS, bitrate=canlib.Bitrate.BITRATE_500K) as ch:
    ch.setBusOutputControl(canlib.Driver.NORMAL)
    ch.busOn()

    ch.writeWait(default_letter, 1000)
    ch.writeWait(default_letter, 1000)
    ch.writeWait(communicate, -1)
    frame = ch.read(timeout=100);

