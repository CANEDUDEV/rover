import argparse
import sys
import time
from pathlib import Path

from canlib import Frame, canlib

from rover import rover

COMMAND_ACK_ID = 0x100
FLASH_PROGRAM_ID = 0x101
ENTER_BOOTLOADER_ID = 0x102
EXIT_BOOTLOADER_ID = 0x103
FLASH_ERASE_ID = 0x104

COMMAND_ACK_FOLDER = 2
FLASH_PROGRAM_TX_FOLDER = 3
ENTER_BOOTLOADER_FOLDER = 4
EXIT_BOOTLOADER_FOLDER = 5
FLASH_ERASE_FOLDER = 6
FLASH_PROGRAM_RX_FOLDER = 7

assignments = [
    (COMMAND_ACK_ID, COMMAND_ACK_FOLDER),
    (FLASH_PROGRAM_ID, FLASH_PROGRAM_TX_FOLDER),
    (ENTER_BOOTLOADER_ID, ENTER_BOOTLOADER_FOLDER),
    (EXIT_BOOTLOADER_ID, EXIT_BOOTLOADER_FOLDER),
    (FLASH_ERASE_ID, FLASH_ERASE_FOLDER),
    (FLASH_PROGRAM_ID, FLASH_PROGRAM_RX_FOLDER),
]


def get_target_city(node):
    if node == "servo":
        return rover.City.SERVO
    if node == "motor":
        return rover.City.MOTOR
    if node == "battery_monitor":
        return rover.City.BATTERY_MONITOR
    if node == "sbus_receiver":
        return rover.City.SBUS_RECEIVER

    return rover.City.ALL_CITIES


def send_bootloader_command(ch, frame, timeout=100):
    ch.iocontrol.flush_rx_buffer()
    ch.write(frame)

    try:
        ch.readSyncSpecific(COMMAND_ACK_ID, timeout=timeout)
        received = ch.readSpecificSkip(COMMAND_ACK_ID)

    except canlib.exceptions.CanNoMsg:
        print("No ACK received. Exiting.")
        sys.exit(1)

    command_id = int.from_bytes(received.data[1:5], byteorder="little")

    if received.data[0] == 1:  # NACK value
        print(f"NACK received for command {hex(command_id)}")
        sys.exit(1)

    if command_id != frame.id:
        print(
            f"Wrong ACK for command, expected: {hex(frame.id)}, got {hex(command_id)}"
        )
        sys.exit(1)


def enter_bootloader():
    return Frame(id_=ENTER_BOOTLOADER_ID, dlc=0, data=[])


def exit_bootloader():
    return Frame(id_=EXIT_BOOTLOADER_ID, dlc=0, data=[])


def is_bootloader_mayor(frame):
    return frame.dlc == 8 and frame.data == bytearray([0, 2, 0, 0, 0, 0, 0, 0])


def flash_erase(bytes_to_erase):
    return Frame(
        id_=FLASH_ERASE_ID,
        dlc=4,
        data=list(bytes_to_erase.to_bytes(4, "little")),
    )


def flash_program(ch, binary_data):
    # init
    data = [1] + list(len(binary_data).to_bytes(4, "little")) + [0, 0, 0]
    ch.write(Frame(id_=FLASH_PROGRAM_ID, dlc=8, data=data))

    try:
        ch.readSyncSpecific(FLASH_PROGRAM_ID, timeout=1000)
        received = ch.readSpecificSkip(FLASH_PROGRAM_ID)

    except canlib.exceptions.CanNoMsg:
        print("No bundle request response (1). Exiting.")
        sys.exit(1)

    if received.data[0] != 2:
        print("Wrong response before programming flash.")
        sys.exit(1)

    if int.from_bytes(received.data[1:3], byteorder="little") != 0xFFFF:
        print("Got wrong bundle request, wanted 0xFFFF.")
        sys.exit(1)

    # Chunk and send data
    chunk_size = 7  # Payload bytes per frame

    chunks = [
        list(binary_data[i : i + chunk_size])
        for i in range(0, len(binary_data), chunk_size)
    ]

    # Pad last chunk if needed
    last_chunk_length = len(chunks[-1])
    chunks[-1] += [0] * (chunk_size - last_chunk_length)

    current_page_number = 3

    for chunk in chunks:
        data = [current_page_number] + chunk
        ch.writeWait(Frame(id_=FLASH_PROGRAM_ID, dlc=8, data=data), -1)

        if current_page_number == 3:
            current_page_number = 4
        else:
            current_page_number = 3

    try:
        ch.readSyncSpecific(FLASH_PROGRAM_ID, timeout=1000)
        received = ch.readSpecificSkip(FLASH_PROGRAM_ID)

    except canlib.exceptions.CanNoMsg:
        print("No bundle request response (2). Exiting.")
        sys.exit(1)

    if received.data[0] != 2:
        print("Wrong response after programming flash.")
        sys.exit(1)

    if int.from_bytes(received.data[1:3], byteorder="little") != 0x0000:
        print("Got wrong bundle request, wanted 0x0000")
        sys.exit(1)


parser = argparse.ArgumentParser(description="Flash Rover firmware over CAN.")

parser.add_argument(
    "target_node",
    choices=["servo", "motor", "battery_monitor", "sbus_receiver"],
    help="Which node to flash.",
)

parser.add_argument("binary_file", help="Binary file to flash.")

args = parser.parse_args()

target_city = get_target_city(args.target_node)

target_file = Path(args.binary_file)
if target_file.suffix != ".bin":
    print(
        f'Incorrect file extension for binary file {target_file}. Please provide a ".bin" file.'
    )
    sys.exit(1)

# Open file and store contents in binary_data
try:
    with open(target_file, "rb") as file:
        binary_data = file.read()
except:
    print(f"Couldn't open {args.binary_file}. Exiting...")
    sys.exit(1)


print("Setting up CAN...")

with canlib.openChannel(
    channel=0,
    flags=canlib.Open.REQUIRE_INIT_ACCESS,
    bitrate=canlib.Bitrate.BITRATE_125K,
) as ch:
    ch.setBusOutputControl(canlib.Driver.NORMAL)
    ch.busOn()

    print("Restarting nodes...")
    ch.writeWait(rover.restart(city=rover.City.ALL_CITIES), -1)

    time.sleep(0.05)  # Wait 50 ms for nodes to restart

    ch.writeWait(rover.default_letter(), -1)

    # All cities should enter bootloader to avoid starting the application
    # during flash. Therefore, we must assign the envelopes for all cities.
    for assignment in assignments:
        envelope = assignment[0]
        folder = assignment[1]
        ch.writeWait(rover.assign_envelope(rover.City.ALL_CITIES, envelope, folder), -1)

    # We only set the target city to communicate mode and assume the other
    # cities have entered the bootloader, without checking their ACK.
    ch.writeWait(rover.set_silent_mode(city=rover.City.ALL_CITIES), -1)
    ch.writeWait(rover.set_communicate(city=target_city), -1)

    # Because all nodes except the target are in silent mode, this command will
    # fail if target node is not found.
    print("Trying to find target node...")
    send_bootloader_command(ch, enter_bootloader())

    ch.writeWait(rover.change_bitrate_1mbit(), -1)

    # Restarting communication means all nodes will be put in silent mode.
    ch.writeWait(rover.restart_communication(skip_startup=True), -1)

    ch.busOff()

# Switch to 1 Mbit/s for flashing
with canlib.openChannel(
    channel=0,
    flags=canlib.Open.REQUIRE_INIT_ACCESS,
    bitrate=canlib.Bitrate.BITRATE_1M,
) as ch:
    ch.setBusOutputControl(canlib.Driver.NORMAL)
    ch.busOn()

    print(f"Starting flash procedure for {target_city.name} node...")

    # Set only city to flash to communicate mode
    ch.writeWait(rover.set_communicate(city=target_city), -1)

    # Erase as many pages as required to fit the application
    print("Erasing target flash...")
    send_bootloader_command(ch, flash_erase(len(binary_data)), timeout=10_000)

    print("Flashing new binary...")
    flash_program(ch, binary_data)

    ch.writeWait(rover.change_bitrate_125kbit(), -1)

    # Restarting communication means all nodes will be put in silent mode.
    ch.writeWait(rover.restart_communication(skip_startup=True), -1)

    ch.busOff()

# Switch back to 125 kbit/s before exiting bootloader.
with canlib.openChannel(
    channel=0,
    flags=canlib.Open.REQUIRE_INIT_ACCESS,
    bitrate=canlib.Bitrate.BITRATE_125K,
) as ch:
    ch.setBusOutputControl(canlib.Driver.NORMAL)
    ch.busOn()

    # Set communicate mode so we can receive ACK
    ch.writeWait(rover.set_communicate(city=target_city), -1)

    send_bootloader_command(ch, exit_bootloader())

    ch.busOff()

print("Finished successfully.")
