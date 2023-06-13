from binascii import hexlify
from canlib import canlib

with canlib.openChannel(channel=0, flags=canlib.Open.NO_INIT_ACCESS) as ch:
    ch.busOn()
    while True:
        try:
            frame = ch.read(timeout=1000)
            print(f'id: {frame.id}, DLC: {frame.dlc}, Data: {hexlify(frame.data, " ")}')
        except canlib.CanNoMsg:
            pass
