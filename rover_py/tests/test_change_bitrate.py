import sys
from time import sleep

from canlib import canlib

from ..rover import rover

try:
    # Start with 125 kbit/s
    with canlib.openChannel(
        channel=0,
        flags=canlib.Open.REQUIRE_INIT_ACCESS,
        bitrate=canlib.Bitrate.BITRATE_125K,
    ) as ch:
        ch.setBusOutputControl(canlib.Driver.NORMAL)
        ch.busOn()

        # Send multiple default letters to make sure every node receives one at startup
        for _ in range(5):
            ch.writeWait(rover.default_letter(), 100)
            sleep(0.1)

        ch.writeWait(rover.give_base_number(), 100)
        frame = ch.read(timeout=1000)
        sleep(0.5)  # Wait for responses

        # Switch to 500 kbit/s
        print("Switching to 500 kbit/s")
        ch.writeWait(rover.change_bitrate_500kbit(), 100)
        ch.writeWait(
            rover.restart_communication(
                skip_startup=True, comm_mode=rover.CommMode.COMMUNICATE
            ),
            -1,
        )

        ch.busOff()
        ch.setBusParams(canlib.Bitrate.BITRATE_500K)
        sleep(0.1)  # give time for changing bitrate
        ch.busOn()

        ch.writeWait(rover.default_letter(), 1000)
        ch.writeWait(rover.give_base_number(), 100)
        frame = ch.read(timeout=1000)
        sleep(0.5)  # Wait for responses

        # print("Switching to 125 kbit/s")
        # ch.writeWait(rover.change_bitrate_125kbit(), 100)
        # ch.writeWait(rover.restart_communication(), 100)

        # ch.busOff()
        # ch.setBusParams(canlib.Bitrate.BITRATE_125K)
        # sleep(0.1)  # give time for changing bitrate
        # ch.busOn()

        # ch.writeWait(rover.default_letter(), 1000)
        # ch.writeWait(rover.give_base_number(), 100)
        # frame = ch.read(timeout=1000)
        # sleep(0.5)  # Wait for responses

        print("Done")

except canlib.exceptions.CanTimeout:
    print(
        "Test timed out. make sure node is on the CAN bus and has a starting bitrate of 125kbit/s",
        file=sys.stderr,
    )
    sys.exit(1)
