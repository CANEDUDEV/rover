import canlib
from canlib import Frame, canlib
from enum import IntEnum

import ck


# City addresses
class City(IntEnum):
    BATTERY_MONITOR = 1
    SERVO = 2
    MOTOR = 3
    SBUS_RECEIVER = 4


# CAN Message IDs
class Envelope(IntEnum):
    # Control messages
    STEERING = 0x100
    THROTTLE = 0x101
    STEERING_TRIM = 0x102
    THROTTLE_TRIM = 0x103

    # Report messages
    BATTERY_CELL_VOLTAGES = 0x200
    BATTERY_REG_OUT_CURRENT = 0x201
    BATTERY_VBAT_OUT_CURRENT = 0x202

    SERVO_VOLTAGE = 0x203
    SERVO_CURRENT = 0x204

    # Settings messages
    BATTERY_JUMPER_AND_FUSE_CONF = 0x300
    BATTERY_REG_OUT_VOLTAGE = 0x301
    BATTERY_OUTPUT_ON_OFF = 0x302
    BATTERY_REPORT_FREQUENCY = 0x303
    BATTERY_LOW_VOLTAGE_CUTOFF = 0x304

    SERVO_SET_VOLTAGE = 0x305
    SERVO_PWM_CONFIG = 0x306
    SERVO_REPORT_FREQUENCY = 0x307

    MOTOR_PWM_CONFIG = 0x308


def get_assign_envelope_frame(city, envelope, folder):
    envelope_data = list(envelope.to_bytes(4, "little"))
    data = [city, 2] + envelope_data + [folder] + [0x3]
    return Frame(id_=0, dlc=8, data=data)


def main():
    # List of assignments in the form (city, envelope, folder)
    kingdom = [
        # Control messages
        (City.SERVO, Envelope.STEERING, 9),
        (City.SERVO, Envelope.STEERING_TRIM, 10),
        (City.MOTOR, Envelope.THROTTLE, 9),
        (City.MOTOR, Envelope.THROTTLE_TRIM, 10),
        (City.SBUS_RECEIVER, Envelope.STEERING, 2),
        (City.SBUS_RECEIVER, Envelope.STEERING_TRIM, 3),
        (City.SBUS_RECEIVER, Envelope.THROTTLE, 4),
        (City.SBUS_RECEIVER, Envelope.THROTTLE_TRIM, 5),
        # Report messages
        (City.BATTERY_MONITOR, Envelope.BATTERY_CELL_VOLTAGES, 2),
        (City.BATTERY_MONITOR, Envelope.BATTERY_REG_OUT_CURRENT, 3),
        (City.BATTERY_MONITOR, Envelope.BATTERY_VBAT_OUT_CURRENT, 4),
        (City.SERVO, Envelope.SERVO_VOLTAGE, 5),
        (City.SERVO, Envelope.SERVO_CURRENT, 3),
        # Settings messages
        (City.BATTERY_MONITOR, Envelope.BATTERY_JUMPER_AND_FUSE_CONF, 5),
        (City.BATTERY_MONITOR, Envelope.BATTERY_REG_OUT_VOLTAGE, 6),
        (City.BATTERY_MONITOR, Envelope.BATTERY_OUTPUT_ON_OFF, 7),
        (City.BATTERY_MONITOR, Envelope.BATTERY_REPORT_FREQUENCY, 8),
        (City.BATTERY_MONITOR, Envelope.BATTERY_LOW_VOLTAGE_CUTOFF, 9),
        (City.SERVO, Envelope.SERVO_SET_VOLTAGE, 7),
        (City.SERVO, Envelope.SERVO_PWM_CONFIG, 8),
        (City.SERVO, Envelope.SERVO_REPORT_FREQUENCY, 11),
        (City.MOTOR, Envelope.MOTOR_PWM_CONFIG, 8),
    ]

    with canlib.openChannel(
        channel=0,
        flags=canlib.Open.REQUIRE_INIT_ACCESS,
        bitrate=canlib.Bitrate.BITRATE_125K,
    ) as ch:
        ch.setBusOutputControl(canlib.Driver.NORMAL)
        ch.busOn()

        ch.writeWait(ck.default_letter, -1)
        ch.writeWait(ck.give_base_no, -1)

        # Assign envelopes
        for assignment in kingdom:
            frame = get_assign_envelope_frame(*assignment)
            ch.writeWait(frame, -1)

        ch.writeWait(ck.communicate, -1)
        ch.busOff()


if __name__ == "__main__":
    main()
