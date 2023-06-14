import ck
import battery

from canlib import canlib

with canlib.openChannel(
    channel=0,
    flags=canlib.Open.REQUIRE_INIT_ACCESS,
    bitrate=canlib.Bitrate.BITRATE_125K,
) as ch:
    ch.setBusOutputControl(canlib.Driver.NORMAL)
    ch.busOn()

    ch.writeWait(ck.default_letter, -1)
    ch.writeWait(ck.give_base_no, -1)
    ch.writeWait(battery.assign_folder2, -1)
    ch.writeWait(battery.assign_folder3, -1)
    ch.writeWait(battery.assign_folder4, -1)
    ch.writeWait(ck.communicate, -1)
    ch.busOff()
