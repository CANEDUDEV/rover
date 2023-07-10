import ck
import battery

from time import sleep
from canlib import canlib, Frame

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

    ch.writeWait(ck.default_letter, -1)
    ch.writeWait(ck.give_base_no, -1)
    ch.writeWait(battery.assign_cell_voltage_tx, -1)
    ch.writeWait(battery.assign_reg_out_current_tx, -1)
    ch.writeWait(battery.assign_vbat_out_current_tx, -1)
    ch.writeWait(ck.communicate, -1)
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

    ch.writeWait(ck.default_letter, 1000)
    ch.writeWait(ck.communicate, -1)
    frame = ch.read(timeout=100)
