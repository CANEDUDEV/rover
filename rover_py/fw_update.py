import argparse
import sys
import time
from pathlib import Path

from canlib import Frame, canlib

from rover import rover

COMMAND_ACK_ID = 0x700
FLASH_PROGRAM_ID = 0x701
ENTER_BOOTLOADER_ID = 0x702
EXIT_BOOTLOADER_ID = 0x703
FLASH_ERASE_ID = 0x704

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

default_timeout_ms = 100


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

    print("Running flasher...")
    try:
        run_flasher(node_binary_map)

    except Exception as err:
        print(f"Error updating firmware: {err}\n")
        raise

    print("Finished successfully.")


def run_flasher(node_binary_map):
    with canlib.openChannel(
        channel=0,
        flags=canlib.Open.REQUIRE_INIT_ACCESS,
        bitrate=canlib.Bitrate.BITRATE_125K,
    ) as ch:
        try:
            Flasher(ch, node_binary_map).run()

        except Exception:
            ch.writeWait(
                rover.set_action_mode(mode=rover.ActionMode.RESET), default_timeout_ms
            )
            raise


class Flasher:
    def __init__(self, ch, node_binary_map):
        self.ch = ch
        self.node_binary_map = node_binary_map
        self.online_nodes = set()

    def run(self):
        self.ch.busOn()
        self.__check_online_nodes()

        for node in self.node_binary_map:
            self.__flash_node(node)

        # Restart all nodes
        self.ch.writeWait(
            rover.set_action_mode(mode=rover.ActionMode.RESET), default_timeout_ms
        )

    def __check_online_nodes(self):
        self.ch.writeWait(
            rover.set_action_mode(mode=rover.ActionMode.FREEZE), default_timeout_ms
        )

        self.ch.iocontrol.flush_rx_buffer()
        self.ch.writeWait(rover.give_base_number(response_page=1), default_timeout_ms)
        time.sleep(0.01)  # Allow time to respond

        # Check responses. Do not procede with flash if the requested nodes have not responded.
        while True:
            try:
                frame = self.ch.read(timeout=10)
                if frame:
                    self.online_nodes.add(frame.id - rover.ROVER_BASE_NUMBER)

            except (canlib.exceptions.CanTimeout, canlib.exceptions.CanNoMsg):
                break

        target_nodes = set(self.node_binary_map.keys())

        if not target_nodes.issubset(self.online_nodes):
            raise RuntimeError(
                f"Error: {target_nodes - self.online_nodes} did not respond."
            )

    def __flash_node(self, node):
        print(f"{node.name}: Starting flash procedure.")

        # Set all nodes except target to silent
        self.ch.write(rover.set_comm_mode(city=node, mode=rover.CommMode.COMMUNICATE))

        for target in self.online_nodes:
            if target != node:
                self.ch.write(
                    rover.set_comm_mode(city=target, mode=rover.CommMode.SILENT)
                )

        # Restart target
        print(f"{node.name}: Entering bootloader...")
        self.ch.writeWait(
            rover.set_action_mode(city=node, mode=rover.ActionMode.RESET),
            default_timeout_ms,
        )

        time.sleep(0.02)  # Wait 20 ms for restart

        # Send default letter. Send should succeed only if node is on the bus again.
        self.ch.writeWait(rover.default_letter(), default_timeout_ms)

        self.__assign_bootloader_envelopes(node)

        # Allow target to communicate
        self.ch.writeWait(
            rover.set_comm_mode(city=node, mode=rover.CommMode.COMMUNICATE),
            default_timeout_ms,
        )

        self.__send_bootloader_command(enter_bootloader())

        # Switch to 1 Mbit/s for flashing and restart communication to apply bitrate
        self.ch.writeWait(rover.change_bitrate_1mbit(), default_timeout_ms)
        self.ch.writeWait(
            rover.restart_communication(
                city=node, skip_startup=True, comm_mode=rover.CommMode.COMMUNICATE
            ),
            default_timeout_ms,
        )

        self.ch.busOff()
        self.ch.setBusParams(canlib.Bitrate.BITRATE_1M)
        time.sleep(0.01)  # Give time for bitrate change
        self.ch.busOn()

        binary = self.node_binary_map[node]
        # Erase as many pages as required to fit the application
        print(f"{node.name}: Erasing target flash...")
        self.__send_bootloader_command(flash_erase(len(binary)), timeout=10_000)

        print(f"{node.name}: Flashing new binary...")
        self.__flash_program(binary)

        # Tell node to exit bootloader
        print(f"{node.name}: Exiting bootloader...")
        self.__send_bootloader_command(exit_bootloader())

        # Go back to 125 kbit/s
        self.ch.busOff()
        self.ch.setBusParams(canlib.Bitrate.BITRATE_125K)
        time.sleep(0.25)
        # The power board powers the other boards, so need to sleep longer to compensate for that case.
        if node is rover.City.BATTERY_MONITOR:
            time.sleep(1)

        self.ch.busOn()

    def __flash_program(self, binary_data):
        # init
        data = [1] + list(len(binary_data).to_bytes(4, "little")) + [0, 0, 0]
        self.ch.write(Frame(id_=FLASH_PROGRAM_ID, dlc=8, data=data))

        try:
            self.ch.readSyncSpecific(FLASH_PROGRAM_ID, timeout=1000)
            received = self.ch.readSpecificSkip(FLASH_PROGRAM_ID)

        except (canlib.exceptions.CanNoMsg, canlib.exceptions.CanTimeout):
            raise RuntimeError("No bundle request response (1). Exiting.")

        if received.data[0] != 2:
            raise RuntimeError("Wrong response before programming flash.")

        if int.from_bytes(received.data[1:3], byteorder="little") != 0xFFFF:
            raise RuntimeError("Got wrong bundle request, wanted 0xFFFF.")

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
            self.ch.write(Frame(id_=FLASH_PROGRAM_ID, dlc=8, data=data))

            if chunk_no % batch_size == 0:
                self.ch.writeSync(timeout=default_timeout_ms)

            if current_page_number == 3:
                current_page_number = 4
            else:
                current_page_number = 3

        self.ch.writeSync(timeout=default_timeout_ms)

        try:
            self.ch.readSyncSpecific(FLASH_PROGRAM_ID, timeout=1000)
            received = self.ch.readSpecificSkip(FLASH_PROGRAM_ID)

        except (canlib.exceptions.CanNoMsg, canlib.exceptions.CanTimeout):
            raise RuntimeError("No bundle request response (2). Exiting.")

        if received.data[0] != 2:
            raise RuntimeError("Wrong response after programming flash.")

        if int.from_bytes(received.data[1:3], byteorder="little") != 0x0000:
            raise RuntimeError("Got wrong bundle request, wanted 0x0000")

    def __assign_bootloader_envelopes(self, node):
        for assignment in assignments:
            envelope = assignment[0]
            folder = assignment[1]
            self.ch.writeWait(
                rover.assign_envelope(node, envelope, folder), default_timeout_ms
            )

    def __send_bootloader_command(self, frame, timeout=default_timeout_ms):
        self.ch.iocontrol.flush_rx_buffer()
        self.ch.write(frame)

        try:
            self.ch.readSyncSpecific(COMMAND_ACK_ID, timeout=timeout)
            received = self.ch.readSpecificSkip(COMMAND_ACK_ID)

        except (canlib.exceptions.CanNoMsg, canlib.exceptions.CanTimeout):
            raise RuntimeError("No ACK received. Exiting.")

        command_id = int.from_bytes(received.data[1:5], byteorder="little")

        if received.data[0] == 1:  # NACK value
            raise RuntimeError(f"NACK received for command {hex(command_id)}")

        if command_id != frame.id:
            raise RuntimeError(
                f"Wrong ACK for command, expected: {hex(frame.id)}, got {hex(command_id)}"
            )


def enter_bootloader():
    return Frame(id_=ENTER_BOOTLOADER_ID, dlc=0, data=[])


def exit_bootloader():
    return Frame(id_=EXIT_BOOTLOADER_ID, dlc=0, data=[])


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
