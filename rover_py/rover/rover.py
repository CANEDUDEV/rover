import enum
from time import sleep

from canlib import Frame

BASE_NUMBER = 0x400


# City addresses
@enum.unique
class City(enum.IntEnum):
    ALL_CITIES = 0
    BATTERY_MONITOR = 1
    SERVO = 2
    MOTOR = 3
    SBUS_RECEIVER = 4
    WHEEL_FRONT_LEFT = 5
    WHEEL_FRONT_RIGHT = 6
    WHEEL_REAR_LEFT = 7
    WHEEL_REAR_RIGHT = 8
    OBSTACLE_DETECTOR_FRONT = 9
    OBSTACLE_DETECTOR_REAR = 10

    # For AD systems
    AD_BATTERY_MONITOR = 100


# CAN Message IDs
@enum.unique
class Envelope(enum.IntEnum):
    # Control messages
    STEERING = 0x100
    THROTTLE = 0x101

    # Report messages
    BATTERY_CELL_VOLTAGES = 0x200
    BATTERY_REGULATED_OUTPUT = 0x201
    BATTERY_OUTPUT = 0x202
    SERVO_VOLTAGE = 0x203
    SERVO_CURRENT = 0x204
    SERVO_POSITION = 0x206
    WHEEL_FRONT_LEFT_SPEED = 0x210
    WHEEL_FRONT_RIGHT_SPEED = 0x211
    WHEEL_REAR_LEFT_SPEED = 0x212
    WHEEL_REAR_RIGHT_SPEED = 0x213
    OBSTACLE_DETECTOR_FRONT_DISTANCE = 0x214
    OBSTACLE_DETECTOR_REAR_DISTANCE = 0x215

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
    STEERING_SUBTRIM = 0x30D
    THROTTLE_SUBTRIM = 0x30E
    BATTERY_VBAT_OUT_OVERCURRENT_THRESHOLD = 0x30F
    BATTERY_REG_OUT_OVERCURRENT_THRESHOLD = 0x310

    WHEEL_FRONT_LEFT_WHEEL_PARAMETERS = 0x311
    WHEEL_FRONT_LEFT_REPORT_FREQUENCY = 0x312

    WHEEL_FRONT_RIGHT_WHEEL_PARAMETERS = 0x313
    WHEEL_FRONT_RIGHT_REPORT_FREQUENCY = 0x314

    WHEEL_REAR_LEFT_WHEEL_PARAMETERS = 0x315
    WHEEL_REAR_LEFT_REPORT_FREQUENCY = 0x316

    WHEEL_REAR_RIGHT_WHEEL_PARAMETERS = 0x317
    WHEEL_REAR_RIGHT_REPORT_FREQUENCY = 0x318

    BATTERY_CELL_CALIBRATION = 0x319

    OBSTACLE_DETECTOR_FRONT_REPORT_FREQUENCY = 0x31A
    OBSTACLE_DETECTOR_REAR_REPORT_FREQUENCY = 0x31B

    # Bootloader specific envelopes
    BOOTLOADER_COMMAND_ACK = 0x700
    BOOTLOADER_ENTER = 0x701
    BOOTLOADER_EXIT = 0x702
    BOOTLOADER_FLASH_ERASE = 0x703
    BOOTLOADER_FORMAT_FS = 0x704
    BOOTLOADER_FLASH_PROGRAM = 0x705
    BOOTLOADER_FLASH_CONFIG = 0x706

    # For AD systems
    AD_BATTERY_CELL_VOLTAGES = 0x500
    AD_BATTERY_REGULATED_OUTPUT = 0x501
    AD_BATTERY_OUTPUT = 0x502
    AD_BATTERY_JUMPER_CONFIG = 0x600
    AD_BATTERY_REG_OUT_VOLTAGE = 0x601
    AD_BATTERY_OUTPUT_ON_OFF = 0x602
    AD_BATTERY_REPORT_FREQUENCY = 0x603
    AD_BATTERY_LOW_VOLTAGE_CUTOFF = 0x604
    AD_BATTERY_VBAT_OUT_OVERCURRENT_THRESHOLD = 0x60F
    AD_BATTERY_REG_OUT_OVERCURRENT_THRESHOLD = 0x610
    AD_BATTERY_CELL_CALIBRATION = 0x611


@enum.unique
class BootloaderFolder(enum.IntEnum):
    COMMAND_ACK = 2
    FLASH_PROGRAM_TX = 3
    FLASH_CONFIG_TX = 4
    ENTER = 5
    EXIT = 6
    FLASH_ERASE = 7
    FORMAT_FS = 8
    FLASH_PROGRAM_RX = 9
    FLASH_CONFIG_RX = 10

    def prefix(self):
        return "BOOTLOADER"


@enum.unique
class ServoFolder(enum.IntEnum):
    POSITION = 2
    CURRENT = 3
    BATTERY_VOLTAGE = 4
    VOLTAGE = 5
    H_BRIDGE_CURRENT = 6
    SET_VOLTAGE = 7
    PWM_CONFIG = 8
    CONTROL = 9
    SET_SUBTRIM = 10
    REPORT_FREQUENCY = 11
    REVERSE_DIRECTION = 12
    FAILSAFE = 13

    def prefix(self):
        return "SERVO"


@enum.unique
class SbusReceiverFolder(enum.IntEnum):
    STEERING = 2
    THROTTLE = 3
    STEERING_SUBTRIM = 4
    THROTTLE_SUBTRIM = 5

    def prefix(self):
        return "SBUS_RECEIVER"


@enum.unique
class BatteryMonitorFolder(enum.IntEnum):
    CELL_VOLTAGES = 2
    REGULATED_OUTPUT = 3
    BATTERY_OUTPUT = 4
    JUMPER_CONFIG = 5
    REG_OUT_VOLTAGE = 6
    OUTPUT_ON_OFF = 7
    REPORT_FREQUENCY = 8
    LOW_VOLTAGE_CUTOFF = 9
    VBAT_OUT_OVERCURRENT_THRESHOLD = 10
    REG_OUT_OVERCURRENT_THRESHOLD = 11
    CELL_CALIBRATION = 12

    def prefix(self):
        return "BATTERY_MONITOR"


@enum.unique
class BrakeFolder(enum.IntEnum):
    WHEEL_SPEED = 2
    WHEEL_PARAMETERS = 3
    REPORT_FREQUENCY = 4

    def prefix(self):
        return "BRAKE"


@enum.unique
class ObstacleDetectorFolder(enum.IntEnum):
    OBJECT_DISTANCE = 2
    REPORT_FREQUENCY = 3

    def prefix(self):
        return "OBSTACLE_DETECTOR"


class Assignment:
    def __init__(self, city, envelope, folder):
        self.city = city
        self.envelope = envelope
        self.folder = folder


SERVO_ASSIGNMENTS = [
    Assignment(
        City.SERVO,
        Envelope.STEERING,
        ServoFolder.CONTROL,
    ),
    Assignment(
        City.SERVO,
        Envelope.SERVO_VOLTAGE,
        ServoFolder.VOLTAGE,
    ),
    Assignment(
        City.SERVO,
        Envelope.SERVO_CURRENT,
        ServoFolder.CURRENT,
    ),
    Assignment(
        City.SERVO,
        Envelope.SERVO_SET_VOLTAGE,
        ServoFolder.SET_VOLTAGE,
    ),
    Assignment(
        City.SERVO,
        Envelope.SERVO_PWM_CONFIG,
        ServoFolder.PWM_CONFIG,
    ),
    Assignment(
        City.SERVO,
        Envelope.SERVO_REPORT_FREQUENCY,
        ServoFolder.REPORT_FREQUENCY,
    ),
    Assignment(
        City.SERVO,
        Envelope.SERVO_REVERSE_DIRECTION,
        ServoFolder.REVERSE_DIRECTION,
    ),
    Assignment(
        City.SERVO,
        Envelope.SERVO_FAILSAFE,
        ServoFolder.FAILSAFE,
    ),
    Assignment(
        City.SERVO,
        Envelope.STEERING_SUBTRIM,
        ServoFolder.SET_SUBTRIM,
    ),
]

MOTOR_ASSIGMENTS = [
    Assignment(
        City.MOTOR,
        Envelope.THROTTLE,
        ServoFolder.CONTROL,
    ),
    Assignment(
        City.MOTOR,
        Envelope.MOTOR_PWM_CONFIG,
        ServoFolder.PWM_CONFIG,
    ),
    Assignment(
        City.MOTOR,
        Envelope.MOTOR_REVERSE_DIRECTION,
        ServoFolder.REVERSE_DIRECTION,
    ),
    Assignment(
        City.MOTOR,
        Envelope.MOTOR_FAILSAFE,
        ServoFolder.FAILSAFE,
    ),
    Assignment(
        City.MOTOR,
        Envelope.THROTTLE_SUBTRIM,
        ServoFolder.SET_SUBTRIM,
    ),
]

SBUS_RECEIVER_ASSIGNMENTS = [
    Assignment(
        City.SBUS_RECEIVER,
        Envelope.STEERING,
        SbusReceiverFolder.STEERING,
    ),
    Assignment(
        City.SBUS_RECEIVER,
        Envelope.THROTTLE,
        SbusReceiverFolder.THROTTLE,
    ),
    Assignment(
        City.SBUS_RECEIVER,
        Envelope.STEERING_SUBTRIM,
        SbusReceiverFolder.STEERING_SUBTRIM,
    ),
    Assignment(
        City.SBUS_RECEIVER,
        Envelope.THROTTLE_SUBTRIM,
        SbusReceiverFolder.THROTTLE_SUBTRIM,
    ),
]

BATTERY_MONITOR_ASSIGNMENTS = [
    Assignment(
        City.BATTERY_MONITOR,
        Envelope.BATTERY_CELL_VOLTAGES,
        BatteryMonitorFolder.CELL_VOLTAGES,
    ),
    Assignment(
        City.BATTERY_MONITOR,
        Envelope.BATTERY_REGULATED_OUTPUT,
        BatteryMonitorFolder.REGULATED_OUTPUT,
    ),
    Assignment(
        City.BATTERY_MONITOR,
        Envelope.BATTERY_OUTPUT,
        BatteryMonitorFolder.BATTERY_OUTPUT,
    ),
    Assignment(
        City.BATTERY_MONITOR,
        Envelope.BATTERY_JUMPER_CONFIG,
        BatteryMonitorFolder.JUMPER_CONFIG,
    ),
    Assignment(
        City.BATTERY_MONITOR,
        Envelope.BATTERY_REG_OUT_VOLTAGE,
        BatteryMonitorFolder.REG_OUT_VOLTAGE,
    ),
    Assignment(
        City.BATTERY_MONITOR,
        Envelope.BATTERY_OUTPUT_ON_OFF,
        BatteryMonitorFolder.OUTPUT_ON_OFF,
    ),
    Assignment(
        City.BATTERY_MONITOR,
        Envelope.BATTERY_REPORT_FREQUENCY,
        BatteryMonitorFolder.REPORT_FREQUENCY,
    ),
    Assignment(
        City.BATTERY_MONITOR,
        Envelope.BATTERY_LOW_VOLTAGE_CUTOFF,
        BatteryMonitorFolder.LOW_VOLTAGE_CUTOFF,
    ),
    Assignment(
        City.BATTERY_MONITOR,
        Envelope.BATTERY_VBAT_OUT_OVERCURRENT_THRESHOLD,
        BatteryMonitorFolder.VBAT_OUT_OVERCURRENT_THRESHOLD,
    ),
    Assignment(
        City.BATTERY_MONITOR,
        Envelope.BATTERY_REG_OUT_OVERCURRENT_THRESHOLD,
        BatteryMonitorFolder.REG_OUT_OVERCURRENT_THRESHOLD,
    ),
    Assignment(
        City.BATTERY_MONITOR,
        Envelope.BATTERY_CELL_CALIBRATION,
        BatteryMonitorFolder.CELL_CALIBRATION,
    ),
]

WHEEL_FRONT_LEFT_ASSIGNMENTS = [
    Assignment(
        City.WHEEL_FRONT_LEFT,
        Envelope.WHEEL_FRONT_LEFT_SPEED,
        BrakeFolder.WHEEL_SPEED,
    ),
    Assignment(
        City.WHEEL_FRONT_LEFT,
        Envelope.WHEEL_FRONT_LEFT_WHEEL_PARAMETERS,
        BrakeFolder.WHEEL_PARAMETERS,
    ),
    Assignment(
        City.WHEEL_FRONT_LEFT,
        Envelope.WHEEL_FRONT_LEFT_REPORT_FREQUENCY,
        BrakeFolder.REPORT_FREQUENCY,
    ),
]

WHEEL_FRONT_RIGHT_ASSIGNMENTS = [
    Assignment(
        City.WHEEL_FRONT_RIGHT,
        Envelope.WHEEL_FRONT_RIGHT_SPEED,
        BrakeFolder.WHEEL_SPEED,
    ),
    Assignment(
        City.WHEEL_FRONT_RIGHT,
        Envelope.WHEEL_FRONT_RIGHT_WHEEL_PARAMETERS,
        BrakeFolder.WHEEL_PARAMETERS,
    ),
    Assignment(
        City.WHEEL_FRONT_RIGHT,
        Envelope.WHEEL_FRONT_RIGHT_REPORT_FREQUENCY,
        BrakeFolder.REPORT_FREQUENCY,
    ),
]

WHEEL_REAR_LEFT_ASSIGNMENTS = [
    Assignment(
        City.WHEEL_REAR_LEFT,
        Envelope.WHEEL_REAR_LEFT_SPEED,
        BrakeFolder.WHEEL_SPEED,
    ),
    Assignment(
        City.WHEEL_REAR_LEFT,
        Envelope.WHEEL_REAR_LEFT_WHEEL_PARAMETERS,
        BrakeFolder.WHEEL_PARAMETERS,
    ),
    Assignment(
        City.WHEEL_REAR_LEFT,
        Envelope.WHEEL_REAR_LEFT_REPORT_FREQUENCY,
        BrakeFolder.REPORT_FREQUENCY,
    ),
]

WHEEL_REAR_RIGHT_ASSIGNMENTS = [
    Assignment(
        City.WHEEL_REAR_RIGHT,
        Envelope.WHEEL_REAR_RIGHT_SPEED,
        BrakeFolder.WHEEL_SPEED,
    ),
    Assignment(
        City.WHEEL_REAR_RIGHT,
        Envelope.WHEEL_REAR_RIGHT_WHEEL_PARAMETERS,
        BrakeFolder.WHEEL_PARAMETERS,
    ),
    Assignment(
        City.WHEEL_REAR_RIGHT,
        Envelope.WHEEL_REAR_RIGHT_REPORT_FREQUENCY,
        BrakeFolder.REPORT_FREQUENCY,
    ),
]

AD_BATTERY_MONITOR_ASSIGNMENTS = [
    Assignment(
        City.AD_BATTERY_MONITOR,
        Envelope.AD_BATTERY_CELL_VOLTAGES,
        BatteryMonitorFolder.CELL_VOLTAGES,
    ),
    Assignment(
        City.AD_BATTERY_MONITOR,
        Envelope.AD_BATTERY_REGULATED_OUTPUT,
        BatteryMonitorFolder.REGULATED_OUTPUT,
    ),
    Assignment(
        City.AD_BATTERY_MONITOR,
        Envelope.AD_BATTERY_OUTPUT,
        BatteryMonitorFolder.BATTERY_OUTPUT,
    ),
    Assignment(
        City.AD_BATTERY_MONITOR,
        Envelope.AD_BATTERY_JUMPER_CONFIG,
        BatteryMonitorFolder.JUMPER_CONFIG,
    ),
    Assignment(
        City.AD_BATTERY_MONITOR,
        Envelope.AD_BATTERY_REG_OUT_VOLTAGE,
        BatteryMonitorFolder.REG_OUT_VOLTAGE,
    ),
    Assignment(
        City.AD_BATTERY_MONITOR,
        Envelope.AD_BATTERY_OUTPUT_ON_OFF,
        BatteryMonitorFolder.OUTPUT_ON_OFF,
    ),
    Assignment(
        City.AD_BATTERY_MONITOR,
        Envelope.AD_BATTERY_REPORT_FREQUENCY,
        BatteryMonitorFolder.REPORT_FREQUENCY,
    ),
    Assignment(
        City.AD_BATTERY_MONITOR,
        Envelope.AD_BATTERY_LOW_VOLTAGE_CUTOFF,
        BatteryMonitorFolder.LOW_VOLTAGE_CUTOFF,
    ),
    Assignment(
        City.AD_BATTERY_MONITOR,
        Envelope.AD_BATTERY_VBAT_OUT_OVERCURRENT_THRESHOLD,
        BatteryMonitorFolder.VBAT_OUT_OVERCURRENT_THRESHOLD,
    ),
    Assignment(
        City.AD_BATTERY_MONITOR,
        Envelope.AD_BATTERY_REG_OUT_OVERCURRENT_THRESHOLD,
        BatteryMonitorFolder.REG_OUT_OVERCURRENT_THRESHOLD,
    ),
    Assignment(
        City.AD_BATTERY_MONITOR,
        Envelope.AD_BATTERY_CELL_CALIBRATION,
        BatteryMonitorFolder.CELL_CALIBRATION,
    ),
]

OBSTACLE_DETECTOR_FRONT_ASSIGNMENTS = [
    Assignment(
        City.OBSTACLE_DETECTOR_FRONT,
        Envelope.OBSTACLE_DETECTOR_FRONT_DISTANCE,
        ObstacleDetectorFolder.OBJECT_DISTANCE,
    ),
    Assignment(
        City.OBSTACLE_DETECTOR_FRONT,
        Envelope.OBSTACLE_DETECTOR_FRONT_REPORT_FREQUENCY,
        ObstacleDetectorFolder.REPORT_FREQUENCY,
    ),
]

OBSTACLE_DETECTOR_REAR_ASSIGNMENTS = [
    Assignment(
        City.OBSTACLE_DETECTOR_REAR,
        Envelope.OBSTACLE_DETECTOR_REAR_DISTANCE,
        ObstacleDetectorFolder.OBJECT_DISTANCE,
    ),
    Assignment(
        City.OBSTACLE_DETECTOR_REAR,
        Envelope.OBSTACLE_DETECTOR_REAR_REPORT_FREQUENCY,
        ObstacleDetectorFolder.REPORT_FREQUENCY,
    ),
]

# List of assignments in the form (city, envelope, folder)
APP_ASSIGNMENTS = (
    SERVO_ASSIGNMENTS
    + MOTOR_ASSIGMENTS
    + SBUS_RECEIVER_ASSIGNMENTS
    + BATTERY_MONITOR_ASSIGNMENTS
    + WHEEL_FRONT_LEFT_ASSIGNMENTS
    + WHEEL_FRONT_RIGHT_ASSIGNMENTS
    + WHEEL_REAR_LEFT_ASSIGNMENTS
    + WHEEL_REAR_RIGHT_ASSIGNMENTS
    + OBSTACLE_DETECTOR_FRONT_ASSIGNMENTS
    + OBSTACLE_DETECTOR_REAR_ASSIGNMENTS
    + AD_BATTERY_MONITOR_ASSIGNMENTS
)


def generate_bootloader_assignments(id):
    return [
        Assignment(id, Envelope.BOOTLOADER_COMMAND_ACK, BootloaderFolder.COMMAND_ACK),
        Assignment(id, Envelope.BOOTLOADER_ENTER, BootloaderFolder.ENTER),
        Assignment(id, Envelope.BOOTLOADER_EXIT, BootloaderFolder.EXIT),
        Assignment(id, Envelope.BOOTLOADER_FLASH_ERASE, BootloaderFolder.FLASH_ERASE),
        Assignment(id, Envelope.BOOTLOADER_FORMAT_FS, BootloaderFolder.FORMAT_FS),
        Assignment(
            id, Envelope.BOOTLOADER_FLASH_PROGRAM, BootloaderFolder.FLASH_PROGRAM_TX
        ),
        Assignment(
            id, Envelope.BOOTLOADER_FLASH_PROGRAM, BootloaderFolder.FLASH_PROGRAM_RX
        ),
        Assignment(
            id, Envelope.BOOTLOADER_FLASH_CONFIG, BootloaderFolder.FLASH_CONFIG_TX
        ),
        Assignment(
            id, Envelope.BOOTLOADER_FLASH_CONFIG, BootloaderFolder.FLASH_CONFIG_RX
        ),
    ]


@enum.unique
class CommMode(enum.IntEnum):
    KEEP_CURRENT = 0
    SILENT = 1
    LISTEN_ONLY = 2
    COMMUNICATE = 3


@enum.unique
class ActionMode(enum.IntEnum):
    KEEP_CURRENT = 0
    RUN = 1
    FREEZE = 2
    RESET = 3


def default_letter():
    return Frame(id_=2031, data=[0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA])


def set_comm_mode(city=City.ALL_CITIES, mode=CommMode.KEEP_CURRENT):
    return Frame(id_=0, dlc=8, data=[city, 0, 0, mode, 0, 0, 0, 0])


def set_action_mode(city=City.ALL_CITIES, mode=ActionMode.KEEP_CURRENT):
    return Frame(id_=0, dlc=8, data=[city, 0, mode, 0, 0, 0, 0, 0])


# Give base number and ask for response page
# Response pages are 0 for EAN and 1 for serial number.
# Besides these two, applications can define their own response pages.
def give_base_number(city=City.ALL_CITIES, base_no=BASE_NUMBER, response_page=0):
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
    for assignment in APP_ASSIGNMENTS:
        channel.writeWait(assign_envelope(*assignment), -1)

    channel.writeWait(set_comm_mode(mode=CommMode.COMMUNICATE), -1)
