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


def main():
    parser = argparse.ArgumentParser(description="Flash Rover firmware over CAN.")

    node_choices = ["servo", "motor", "battery_monitor", "sbus_receiver"]
    for node in node_choices:
        parser.add_argument(
            f"--{node}",
            dest=node,
            metavar=f"{node.upper()}_FILE",
            help=f"Binary file for {node} node.",
        )

    args = parser.parse_args()

    print("Checking binaries...")
    node_binary_map = {}
    for node in node_choices:
        file_path = getattr(args, node)
        if not file_path:
            continue

        verify_file(file_path)
        node_binary_map[get_node_id(node)] = read_file(file_path)

    if not node_binary_map:
        parser.print_usage()
        sys.exit(0)

    prepare_flash(node_binary_map.keys())

    for node, binary in node_binary_map.items():
        flash_node(node, binary)

    finalize_flash()

    print("Finished successfully.")


def prepare_flash(node_ids):
    with canlib.openChannel(
        channel=0,
        flags=canlib.Open.REQUIRE_INIT_ACCESS,
        bitrate=canlib.Bitrate.BITRATE_125K,
    ) as ch:
        ch.setBusOutputControl(canlib.Driver.NORMAL)
        ch.busOn()

        print("Restarting nodes...")
        ch.writeWait(rover.restart(), -1)

        time.sleep(0.01)  # Wait 10 ms for restart

        # Send several default letters to make sure every node has had time to start up and receives at least one
        for _ in range(5):
            ch.writeWait(rover.default_letter(), -1)
            time.sleep(0.01)

        check_online_nodes(ch, node_ids)

        assign_bootloader_envelopes(ch)

        # Set communicate mode so we can receive ACKs
        ch.writeWait(rover.set_communicate(), -1)

        # All nodes will try to ACK on the same ID. It's OK if at least one ACK is received.
        send_bootloader_command(ch, enter_bootloader())

        # Switch to 1 Mbit/s for flashing
        ch.writeWait(rover.change_bitrate_1mbit(), -1)

        # Restarting communication means all nodes will be put in silent mode.
        ch.writeWait(rover.restart_communication(skip_startup=True), -1)

        ch.busOff()

    time.sleep(0.01)  # Give time for bitrate change


def flash_node(node, binary_data):
    with canlib.openChannel(
        channel=0,
        flags=canlib.Open.REQUIRE_INIT_ACCESS,
        bitrate=canlib.Bitrate.BITRATE_1M,
    ) as ch:
        ch.setBusOutputControl(canlib.Driver.NORMAL)
        ch.busOn()

        print(f"Starting flash procedure for {node.name} node...")

        # Set target to communicate
        ch.writeWait(rover.set_communicate(city=node), -1)

        # Erase as many pages as required to fit the application
        print("Erasing target flash...")
        send_bootloader_command(ch, flash_erase(len(binary_data)), timeout=10_000)

        print("Flashing new binary...")
        flash_program(ch, binary_data)

        # Go back to silent for this target
        ch.writeWait(rover.set_silent_mode(city=node), -1)
        ch.busOff()


# Switch back to 125 kbit/s, then exit bootloader.
def finalize_flash():
    with canlib.openChannel(
        channel=0,
        flags=canlib.Open.REQUIRE_INIT_ACCESS,
        bitrate=canlib.Bitrate.BITRATE_1M,
    ) as ch:
        ch.setBusOutputControl(canlib.Driver.NORMAL)
        ch.busOn()

        # Set all to communicate to avoid error frames
        ch.writeWait(rover.set_communicate(), -1)
        ch.writeWait(rover.change_bitrate_125kbit(), -1)

        # Restarting communication means all nodes will be put in silent mode.
        ch.writeWait(rover.restart_communication(skip_startup=True), -1)

    time.sleep(0.01)  # Give time for bitrate change

    with canlib.openChannel(
        channel=0,
        flags=canlib.Open.REQUIRE_INIT_ACCESS,
        bitrate=canlib.Bitrate.BITRATE_125K,
    ) as ch:
        ch.setBusOutputControl(canlib.Driver.NORMAL)
        ch.busOn()

        # Set communicate mode so we can receive ACK
        ch.writeWait(rover.set_communicate(), -1)

        send_bootloader_command(ch, exit_bootloader())

        ch.busOff()


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
    batch_size = 128  # Set to avoid TX buffer overflow

    for chunk_no, chunk in enumerate(chunks):
        data = [current_page_number] + chunk
        ch.write(Frame(id_=FLASH_PROGRAM_ID, dlc=8, data=data))

        if chunk_no % batch_size == 0:
            ch.writeSync(timeout=50)

        if current_page_number == 3:
            current_page_number = 4
        else:
            current_page_number = 3

    ch.writeSync(timeout=100)

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


def check_online_nodes(ch, node_ids):
    ch.iocontrol.flush_rx_buffer()
    ch.writeWait(rover.give_base_number(response_page=1), -1)
    time.sleep(0.01)  # Allow time to respond

    # Check responses. Do not procede with flash if the requested nodes have not responded.
    response_ids = []

    while True:
        try:
            frame = ch.read(timeout=10)
            if frame:
                response_ids.append(frame.id - rover.ROVER_BASE_NUMBER)

        except (canlib.exceptions.CanTimeout, canlib.exceptions.CanNoMsg):
            break

    for id in node_ids:
        if id not in response_ids:
            print(
                f"Error: {rover.City(id).name} node's ID ({id}) was not found in responses: {response_ids}"
            )
            sys.exit(1)


def assign_bootloader_envelopes(ch):
    # All cities should enter bootloader to avoid starting the application
    # during flash. Therefore, we must assign the envelopes for all cities.
    for assignment in assignments:
        envelope = assignment[0]
        folder = assignment[1]
        ch.writeWait(rover.assign_envelope(rover.City.ALL_CITIES, envelope, folder), -1)


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


def get_node_id(node):
    cities = {
        "servo": rover.City.SERVO,
        "motor": rover.City.MOTOR,
        "battery_monitor": rover.City.BATTERY_MONITOR,
        "sbus_receiver": rover.City.SBUS_RECEIVER,
    }
    return cities[node]


def verify_file(file):
    file_path = Path(file)
    if not file_path.exists():
        print(f'error: file "{file}" not found.')
        sys.exit(1)

    if file_path.suffix != ".bin":
        print(
            f'Incorrect file extension for binary file {file}. Please provide a ".bin" file.'
        )
        sys.exit(1)


def read_file(file):
    try:
        with open(Path(file), "rb") as f:
            return f.read()
    except:
        print(f"Couldn't open {file}. Exiting...")
        sys.exit(1)


if __name__ == "__main__":
    main()
