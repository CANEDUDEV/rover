import ck
import battery

from canlib import canlib, Frame
from time import sleep

set_20ma_current = Frame(
    id_=0x103, dlc=7, data=[0xFF, 0xFF, 0x01, 20, 0, 0, 0, 0]
)  # Set over-current threshold to 20 mA. All other values are ignored.

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
    ch.writeWait(battery.assign_folder5, -1)
    ch.writeWait(ck.communicate, -1)
    sleep(5)
    # This should turn off the power out
    ch.writeWait(set_20ma_current, -1)
