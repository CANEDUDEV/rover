import sys
from binascii import hexlify

from canlib import canlib

flags = canlib.Open.NO_INIT_ACCESS
if len(sys.argv) > 1 and sys.argv[1] == "set-bitrate":
    flags = canlib.Open.REQUIRE_INIT_ACCESS

with canlib.openChannel(channel=0, flags=flags, bitrate=canlib.canBITRATE_125K) as ch:
    ch.busOn()
    while True:
        try:
            frame = ch.read(timeout=1000)
            print(
                f'{frame.timestamp} id: {frame.id}, DLC: {frame.dlc}, Data: {hexlify(frame.data, " ")}'
            )
        except canlib.CanNoMsg:
            pass
