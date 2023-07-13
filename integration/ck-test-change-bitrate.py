from time import sleep
from canlib import canlib, Frame
from rover import Rover

change_bitrate = Frame(
    id_=0, dlc=8, data=[0, 8, 0, 0, 9, 8, 1, 1]
)  # Set bitrate to 500kbit/s

reset = Frame(
    id_=0, dlc=8, data=[0, 0, 0, 0x55, 0, 0, 0, 0]
)  # Reset communication and set communication mode to SILENT

with canlib.openChannel(
    channel=0,
    flags=canlib.Open.REQUIRE_INIT_ACCESS,
    bitrate=canlib.Bitrate.BITRATE_125K,
) as ch:
    ch.setBusOutputControl(canlib.Driver.NORMAL)
    ch.busOn()
    rover = Rover(ch)
    rover.start()

    sleep(0.5)  # give time for communication
    ch.writeWait(change_bitrate, -1)
    ch.writeWait(reset, -1)

    ch.busOff()

with canlib.openChannel(
    channel=0,
    flags=canlib.Open.REQUIRE_INIT_ACCESS,
    bitrate=canlib.Bitrate.BITRATE_500K,
) as ch:
    ch.setBusOutputControl(canlib.Driver.NORMAL)
    ch.busOn()
    rover = Rover(ch)

    ch.writeWait(rover.default_letter, 1000)
    ch.writeWait(rover.communicate, -1)
    frame = ch.read(timeout=100)

    ch.busOff()
