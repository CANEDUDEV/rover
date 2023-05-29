from canlib import canlib, Frame

with canlib.openChannel(channel=0, flags=canlib.Open.REQUIRE_INIT_ACCESS, bitrate=canlib.Bitrate.BITRATE_125K) as ch:
    ch.setBusOutputControl(canlib.Driver.NORMAL)
    ch.busOn()
    default_letter = Frame(id_=2031, data=[0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA])
    kp0 = Frame(id_=0, dlc=8, data=[0,0,0,0x3,0,0,0,0]) # Set communication mode to COMMUNICATE
    kp1 = Frame(id_=0, dlc=8, data=[0,1,0,0,0,2,0,0]) # Give base number and ask for response page 0
    kp2_0 = Frame(id_=0, dlc=8, data=[0,2,2,0,0,0,2,0x3]) # Assign envelope 2 to folder 2
    kp2_1 = Frame(id_=0, dlc=8, data=[0,2,3,0,0,0,3,0x3]) # Assign envelope 3 to folder 3
    kp2_2 = Frame(id_=0, dlc=8, data=[0,2,4,0,0,0,4,0x3]) # Assign envelope 4 to folder 4

    ch.writeWait(default_letter, -1)
    ch.writeWait(kp1, -1)
    ch.writeWait(kp2_0, -1)
    ch.writeWait(kp2_1, -1)
    ch.writeWait(kp2_2, -1)
    ch.writeWait(kp0, -1)
