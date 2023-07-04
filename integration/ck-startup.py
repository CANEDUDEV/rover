import ck
import battery
import sbus
import servo

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
    ch.writeWait(battery.assign_folder5, -1)
    ch.writeWait(battery.assign_folder6, -1)
    ch.writeWait(battery.assign_folder7, -1)
    ch.writeWait(battery.assign_folder8, -1)
    ch.writeWait(battery.assign_folder9, -1)

    ch.writeWait(servo.assign_folder2, -1)
    ch.writeWait(servo.assign_folder3, -1)
    ch.writeWait(servo.assign_folder4, -1)
    ch.writeWait(servo.assign_folder5, -1)
    ch.writeWait(servo.assign_folder6, -1)
    ch.writeWait(servo.assign_folder7, -1)
    ch.writeWait(servo.assign_folder8, -1)
    ch.writeWait(servo.assign_folder9, -1)
    ch.writeWait(servo.assign_folder10, -1)

    ch.writeWait(sbus.assign_folder2, -1)
    ch.writeWait(sbus.assign_folder3, -1)
    ch.writeWait(sbus.assign_folder4, -1)
    ch.writeWait(sbus.assign_folder5, -1)
    ch.writeWait(sbus.assign_steering, -1)

    ch.writeWait(ck.communicate, -1)
    ch.busOff()
