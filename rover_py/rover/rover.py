from enum import IntEnum
from time import sleep

from canlib import Frame


# City addresses
class City(IntEnum):
    ALL_CITIES = 0
    BATTERY_MONITOR = 1
    SERVO = 2
    MOTOR = 3
    SBUS_RECEIVER = 4


class CommMode(IntEnum):
    KEEP_CURRENT = 0
    SILENT = 1
    LISTEN_ONLY = 2
    COMMUNICATE = 3


class ActionMode(IntEnum):
    KEEP_CURRENT = 0
    RUN = 1
    FREEZE = 2
    RESET = 3


ROVER_BASE_NUMBER = 0x400


# CAN Message IDs
class Envelope(IntEnum):
    # Control messages
    STEERING = 0x100
    THROTTLE = 0x101

    # Report messages
    BATTERY_CELL_VOLTAGES = 0x200
    BATTERY_REGULATED_OUTPUT_ENVELOPE = 0x201
    BATTERY_BATTERY_OUTPUT_ENVELOPE = 0x202
    SERVO_VOLTAGE = 0x203
    SERVO_CURRENT = 0x204
    BATTERY_VOLTAGE = 0x205  # This is temporary until power board is on Rover.
    SERVO_POSITION = 0x206

    # Settings messages
    BATTERY_JUMPER_CONFIG = 0x300
    BATTERY_REG_OUT_VOLTAGE = 0x301
    BATTERY_OUTPUT_ON_OFF = 0x302
    BATTERY_REPORT_FREQUENCY = 0x303
    BATTERY_LOW_VOLTAGE_CUTOFF = 0x304
    SERVO_SET_VOLTAGE = 0x305
    SERVO_PWM_CONFIG = 0x306
    SERVO_REPORT_FREQUENCY = 0x307
    MOTOR_PWM_CONFIG = 0x308
    SERVO_REVERSE_DIRECTION = 0x309
    MOTOR_REVERSE_DIRECTION = 0x30A
    SERVO_FAILSAFE = 0x30B
    MOTOR_FAILSAFE = 0x30C
    SERVO_SET_SUBTRIM_ENVELOPE = 0x30D
    MOTOR_SET_SUBTRIM_ENVELOPE = 0x30E
    BATTERY_VBAT_OUT_OVERCURRENT_THRESHOLD = 0x30F
    BATTERY_REG_OUT_OVERCURRENT_THRESHOLD = 0x310


# List of assignments in the form (city, envelope, folder)
KINGDOM_ASSIGNMENTS = [
    # Control messages
    (City.SERVO, Envelope.STEERING, 9),
    (City.MOTOR, Envelope.THROTTLE, 9),
    (City.SBUS_RECEIVER, Envelope.STEERING, 2),
    (City.SBUS_RECEIVER, Envelope.THROTTLE, 3),
    # Report messages
    (City.BATTERY_MONITOR, Envelope.BATTERY_CELL_VOLTAGES, 2),
    (City.BATTERY_MONITOR, Envelope.BATTERY_REGULATED_OUTPUT_ENVELOPE, 3),
    (City.BATTERY_MONITOR, Envelope.BATTERY_BATTERY_OUTPUT_ENVELOPE, 4),
    (City.SERVO, Envelope.SERVO_VOLTAGE, 5),
    (City.SERVO, Envelope.SERVO_CURRENT, 3),
    # Don't enable this by default
    # (City.SERVO, Envelope.SERVO_POSITION, 2),
    # Settings messages
    (City.BATTERY_MONITOR, Envelope.BATTERY_JUMPER_CONFIG, 5),
    (City.BATTERY_MONITOR, Envelope.BATTERY_REG_OUT_VOLTAGE, 6),
    (City.BATTERY_MONITOR, Envelope.BATTERY_OUTPUT_ON_OFF, 7),
    (City.BATTERY_MONITOR, Envelope.BATTERY_REPORT_FREQUENCY, 8),
    (City.BATTERY_MONITOR, Envelope.BATTERY_LOW_VOLTAGE_CUTOFF, 9),
    (City.BATTERY_MONITOR, Envelope.BATTERY_VBAT_OUT_OVERCURRENT_THRESHOLD, 10),
    (City.BATTERY_MONITOR, Envelope.BATTERY_REG_OUT_OVERCURRENT_THRESHOLD, 11),
    (City.SERVO, Envelope.SERVO_SET_VOLTAGE, 7),
    (City.SERVO, Envelope.SERVO_PWM_CONFIG, 8),
    (City.SERVO, Envelope.SERVO_REPORT_FREQUENCY, 11),
    (City.MOTOR, Envelope.MOTOR_PWM_CONFIG, 8),
    (City.SERVO, Envelope.SERVO_REVERSE_DIRECTION, 12),
    (City.MOTOR, Envelope.MOTOR_REVERSE_DIRECTION, 12),
    (City.SERVO, Envelope.SERVO_FAILSAFE, 13),
    (City.MOTOR, Envelope.MOTOR_FAILSAFE, 13),
    (City.SERVO, Envelope.SERVO_SET_SUBTRIM_ENVELOPE, 10),
    (City.MOTOR, Envelope.MOTOR_SET_SUBTRIM_ENVELOPE, 10),
]


def default_letter():
    return Frame(id_=2031, data=[0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA])


def set_comm_mode(city=City.ALL_CITIES, mode=CommMode.KEEP_CURRENT):
    return Frame(id_=0, dlc=8, data=[city, 0, 0, mode, 0, 0, 0, 0])


def set_action_mode(city=City.ALL_CITIES, mode=ActionMode.KEEP_CURRENT):
    return Frame(id_=0, dlc=8, data=[city, 0, mode, 0, 0, 0, 0, 0])


# Give base number and ask for response page
# Response pages are 0 for EAN and 1 for serial number.
# Besides these two, applications can define their own response pages.
def give_base_number(city=City.ALL_CITIES, base_no=ROVER_BASE_NUMBER, response_page=0):
    base_no_data = list(base_no.to_bytes(4, "little"))
    data = [city, 1, response_page, 0] + base_no_data
    return Frame(id_=0, dlc=8, data=data)


def assign_envelope(city, envelope, folder):
    envelope_data = list(envelope.to_bytes(4, "little"))
    data = [city, 2] + envelope_data + [folder, 0x3]
    return Frame(id_=0, dlc=8, data=data)


def change_bit_timing(prescaler, tq, phase_seg2, sjw, city=City.ALL_CITIES):
    return Frame(id_=0, dlc=8, data=[city, 8, 0, 0, prescaler, tq, phase_seg2, sjw])


def change_bitrate_125kbit(city=City.ALL_CITIES):
    return change_bit_timing(prescaler=18, tq=16, phase_seg2=2, sjw=1, city=city)


def change_bitrate_500kbit(city=City.ALL_CITIES):
    return change_bit_timing(prescaler=9, tq=8, phase_seg2=1, sjw=1, city=city)


def change_bitrate_1mbit(city=City.ALL_CITIES):
    return change_bit_timing(prescaler=4, tq=9, phase_seg2=1, sjw=1, city=city)


# Reset communication mode. Useful when switching bitrates.
# Optionally skip startup sequence after reset.
def restart_communication(
    city=City.ALL_CITIES, skip_startup=False, comm_mode=CommMode.SILENT
):
    frame = set_comm_mode(city=city, mode=comm_mode)
    # See CanKingdom v4 KP0 for definition
    reset_flag = 1 << 2
    if skip_startup:
        skip_listen_flag = 1 << 3
        skip_wait_flag = 1 << 5
    else:
        skip_listen_flag = 1 << 4
        skip_wait_flag = 1 << 6

    frame.data[3] |= reset_flag
    frame.data[3] |= skip_listen_flag
    frame.data[3] |= skip_wait_flag

    return frame


# Start the kingdom
def start(channel):
    if channel is None:
        return

    # Send several default letters to make sure every node has had time to start up and receives at least one
    for _ in range(5):
        channel.writeWait(default_letter(), -1)
        sleep(0.1)

    channel.writeWait(give_base_number(response_page=1), -1)
    sleep(0.1)  # Allow time to respond

    # Assign envelopes
    for assignment in KINGDOM_ASSIGNMENTS:
        channel.writeWait(assign_envelope(*assignment), -1)

    channel.writeWait(set_comm_mode(mode=CommMode.COMMUNICATE), -1)
