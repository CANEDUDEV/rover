from time import sleep

from canlib import Frame, canlib

from ..rover.rover import Rover

change_bitrate_125kbit = Frame(
    id_=0, dlc=8, data=[0, 8, 0, 0, 18, 16, 2, 1]
)  # Set bitrate to 125kbit/s

change_bitrate_500kbit = Frame(
    id_=0, dlc=8, data=[0, 8, 0, 0, 9, 8, 1, 1]
)  # Set bitrate to 500kbit/s

reset = Frame(
    id_=0, dlc=8, data=[0, 0, 0, 0x55, 0, 0, 0, 0]
)  # Reset communication and set communication mode to SILENT

# Start with 125 kbit/s
with canlib.openChannel(
    channel=0,
    flags=canlib.Open.REQUIRE_INIT_ACCESS,
    bitrate=canlib.Bitrate.BITRATE_125K,
) as ch:
    ch.setBusOutputControl(canlib.Driver.NORMAL)
    ch.busOn()
    rover = Rover(ch)

    # Send multiple default letters to make sure every node receives one at startup
    for _ in range(5):
        ch.writeWait(rover.default_letter, -1)
        sleep(0.1)

    ch.writeWait(rover.give_base_number, -1)
    frame = ch.read(timeout=1000)
    sleep(0.5)  # Wait for responses

    ch.writeWait(change_bitrate_500kbit, -1)
    ch.writeWait(reset, -1)

    ch.busOff()


print("Switching to 500 kbit/s")
sleep(0.1)  # give time for changing bitrate

# Switch to 500 kbit/s
with canlib.openChannel(
    channel=0,
    flags=canlib.Open.REQUIRE_INIT_ACCESS,
    bitrate=canlib.Bitrate.BITRATE_500K,
) as ch:
    ch.setBusOutputControl(canlib.Driver.NORMAL)
    ch.busOn()
    rover = Rover(ch)

    ch.writeWait(rover.default_letter, 1000)
    ch.writeWait(rover.give_base_number, -1)
    frame = ch.read(timeout=1000)
    sleep(0.5)  # Wait for responses

    ch.writeWait(change_bitrate_125kbit, -1)
    ch.writeWait(reset, -1)

    ch.busOff()

print("Switching to 125 kbit/s")
sleep(0.1)  # give time for changing bitrate

# Go back to 125 kbit/s so it is saved in persistent memory
with canlib.openChannel(
    channel=0,
    flags=canlib.Open.REQUIRE_INIT_ACCESS,
    bitrate=canlib.Bitrate.BITRATE_125K,
) as ch:
    ch.setBusOutputControl(canlib.Driver.NORMAL)
    ch.busOn()
    rover = Rover(ch)

    ch.writeWait(rover.default_letter, 1000)
    ch.writeWait(rover.give_base_number, -1)
    frame = ch.read(timeout=1000)
    sleep(0.5)  # Wait for responses

    ch.busOff()

print("Done ")
