from canlib import canlib, Frame

default_letter = Frame(id_=2031, data=[0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA])
give_base_no = Frame(id_=0, dlc=8, data=[0,1,0,0,0,2,0,0]) # Give base number and ask for response page 0
assign_folder2 = Frame(id_=0, dlc=8, data=[0,2,0,1,0,0,2,0x3]) # Assign envelope 0x100 to folder 2
assign_folder3 = Frame(id_=0, dlc=8, data=[0,2,1,1,0,0,3,0x3]) # Assign envelope 0x101 to folder 3
assign_folder4 = Frame(id_=0, dlc=8, data=[0,2,2,1,0,0,4,0x3]) # Assign envelope 0x102 to folder 4
communicate = Frame(id_=0, dlc=8, data=[0,0,0,0x3,0,0,0,0]) # Set communication mode to COMMUNICATE

with canlib.openChannel(channel=0, flags=canlib.Open.REQUIRE_INIT_ACCESS, bitrate=canlib.Bitrate.BITRATE_125K) as ch:
    ch.setBusOutputControl(canlib.Driver.NORMAL)
    ch.busOn()

    ch.writeWait(default_letter, -1)
    ch.writeWait(give_base_no, -1)
    ch.writeWait(assign_folder2, -1)
    ch.writeWait(assign_folder3, -1)
    ch.writeWait(assign_folder4, -1)
    ch.writeWait(communicate, -1)
    ch.busOff()
