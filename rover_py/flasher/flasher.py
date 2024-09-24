import json
import time
from pathlib import Path

from canlib import Frame, canlib

from rover import rover

COMMAND_ACK_ID = 0x700
ENTER_BOOTLOADER_ID = 0x701
EXIT_BOOTLOADER_ID = 0x702
FLASH_ERASE_ID = 0x703
FORMAT_FS_ID = 0x704
FLASH_PROGRAM_ID = 0x705
FLASH_CONFIG_ID = 0x706

COMMAND_ACK_FOLDER = 2
FLASH_PROGRAM_TX_FOLDER = 3
FLASH_CONFIG_TX_FOLDER = 4
ENTER_BOOTLOADER_FOLDER = 5
EXIT_BOOTLOADER_FOLDER = 6
FLASH_ERASE_FOLDER = 7
FORMAT_FS_FOLDER = 8
FLASH_PROGRAM_RX_FOLDER = 9
FLASH_CONFIG_RX_FOLDER = 10

assignments = [
    (COMMAND_ACK_ID, COMMAND_ACK_FOLDER),
    (ENTER_BOOTLOADER_ID, ENTER_BOOTLOADER_FOLDER),
    (EXIT_BOOTLOADER_ID, EXIT_BOOTLOADER_FOLDER),
    (FLASH_ERASE_ID, FLASH_ERASE_FOLDER),
    (FORMAT_FS_ID, FORMAT_FS_FOLDER),
    (FLASH_PROGRAM_ID, FLASH_PROGRAM_TX_FOLDER),
    (FLASH_PROGRAM_ID, FLASH_PROGRAM_RX_FOLDER),
    (FLASH_CONFIG_ID, FLASH_CONFIG_TX_FOLDER),
    (FLASH_CONFIG_ID, FLASH_CONFIG_RX_FOLDER),
]


class Flasher:
    def __init__(self, ch, config=None):
        self.ch = ch
        self.config = config
        self.online_node_ids = set()
        self.default_timeout_ms = 100
        self.ch.busOn()

        try:
            self.__detect_online_nodes()
        except (Exception, canlib.exceptions.CanlibException):
            raise

    def run(self):
        if not self.config:
            raise ValueError(f"run method requires config parameter")

        # Flash all online nodes
        for node in self.config.get_nodes():
            id = self.config.get_id(node)
            binary = self.config.get_binary(node)
            config = self.config.get_config(node)
            if id in self.online_node_ids:
                self.__flash_node(id, node=node, binary=binary, config=config)

        self.__restart_all()

    def run_single(self, id, binary_file=None, config_file=None):
        if id not in self.online_node_ids:
            raise ValueError(
                f"node {id}: node is offline. Found: {self.online_node_ids}"
            )

        binary = None
        config = None

        if binary_file:
            binary = _read_binary(binary_file)

        if config_file:
            config = _read_json(config_file)

        self.__flash_node(id, binary=binary, config=config)
        self.__restart_all()

    def format_fs(self, id):
        prefix = f"node {id}"
        if id not in self.online_node_ids:
            raise ValueError(
                f"{prefix}: node is offline. Found: {self.online_node_ids}"
            )

        print(f"{prefix}: Starting FS format procedure.")
        self.__single_comm_mode(id)

        print(f"{prefix}: Entering bootloader...")
        self.__enter_bootloader(id)

        print(f"{prefix}: formatting FS...")
        self.__format_fs()

        print(f"{prefix}: Exiting bootloader...")
        self.__exit_bootloader()

    def __flash_node(self, id, node=None, binary=None, config=None):
        if not binary and not config:
            raise ValueError("at least one of binary and config params must be set")

        prefix = f"node {id}"
        if node:
            prefix = node

        print(f"{prefix}: Starting flash procedure.")
        self.__single_comm_mode(id)

        print(f"{prefix}: Entering bootloader...")
        self.__enter_bootloader(id)

        # Switch to 1 Mbit/s for flashing and restart communication to apply bitrate
        self.ch.write(rover.change_bitrate_1mbit())
        self.ch.writeWait(
            rover.restart_communication(
                city=id, skip_startup=True, comm_mode=rover.CommMode.COMMUNICATE
            ),
            self.default_timeout_ms,
        )

        self.ch.busOff()
        self.ch.setBusParams(canlib.Bitrate.BITRATE_1M)
        time.sleep(0.1)  # Give time for bitrate change
        self.ch.busOn()

        if binary:
            # Erase as many pages as required to fit the application
            print(f"{prefix}: Erasing target flash...")
            self.__flash_erase(len(binary))

            print(f"{prefix}: Flashing new binary...")
            self.__flash_program(binary)

        if config:
            print(f"{prefix}: Writing config...")
            self.__write_config(config)

        print(f"{prefix}: Exiting bootloader...")
        self.__exit_bootloader()

        # Go back to 125 kbit/s
        self.ch.busOff()
        self.ch.setBusParams(canlib.Bitrate.BITRATE_125K)
        time.sleep(0.25)

        # The power board powers the other boards, so need to sleep longer to compensate for that case.
        if id is rover.City.BATTERY_MONITOR:
            time.sleep(1)

        self.ch.busOn()

    def __block_transfer(self, envelope, binary):
        # Init block transfer
        data = [1] + list(len(binary).to_bytes(4, "little")) + [0, 0, 0]
        self.ch.write(Frame(id_=envelope, dlc=8, data=data))

        try:
            self.ch.readSyncSpecific(envelope, timeout=1000)
            received = self.ch.readSpecificSkip(envelope)

        except (canlib.exceptions.CanNoMsg, canlib.exceptions.CanTimeout):
            raise RuntimeError("No bundle request response (1). Exiting.")

        if received.data[0] != 2:
            raise RuntimeError("Wrong response during block transfer init.")

        if int.from_bytes(received.data[1:3], byteorder="little") != 0xFFFF:
            raise RuntimeError("Got wrong bundle request, wanted 0xFFFF.")

        # Chunk and send data
        chunk_size = 7  # Payload bytes per frame

        chunks = [
            list(binary[i : i + chunk_size]) for i in range(0, len(binary), chunk_size)
        ]

        # Pad last chunk if needed
        last_chunk_length = len(chunks[-1])
        chunks[-1] += [0] * (chunk_size - last_chunk_length)

        current_page_number = 3
        batch_size = 128  # Set to avoid TX buffer overflow

        try:
            for chunk_no, chunk in enumerate(chunks):
                data = [current_page_number] + chunk
                self.ch.write(Frame(id_=envelope, dlc=8, data=data))

                if chunk_no % batch_size == 0:
                    self.ch.writeSync(timeout=self.default_timeout_ms)

                if current_page_number == 3:
                    current_page_number = 4
                else:
                    current_page_number = 3

            self.ch.writeSync(timeout=self.default_timeout_ms)
        except canlib.exceptions.CanTimeout as e:
            raise RuntimeError(f"Block transfer timed out") from e

        try:
            self.ch.readSyncSpecific(envelope, timeout=5000)
            received = self.ch.readSpecificSkip(envelope)

        except (canlib.exceptions.CanNoMsg, canlib.exceptions.CanTimeout):
            raise RuntimeError("No bundle request response (2). Exiting.")

        if received.data[0] != 2:
            raise RuntimeError("Wrong response after block transfer")

        if int.from_bytes(received.data[1:3], byteorder="little") != 0x0000:
            raise RuntimeError("Got wrong bundle request, wanted 0x0000")

    def __detect_online_nodes(self):
        self.ch.writeWait(
            rover.set_action_mode(mode=rover.ActionMode.FREEZE), self.default_timeout_ms
        )

        self.ch.iocontrol.flush_rx_buffer()
        self.ch.writeWait(
            rover.give_base_number(response_page=1), self.default_timeout_ms
        )
        time.sleep(0.1)  # Allow time to respond

        # Check responses
        while True:
            try:
                frame = self.ch.read(timeout=10)
                if frame:
                    self.online_node_ids.add(frame.id - rover.ROVER_BASE_NUMBER)

            except (canlib.exceptions.CanTimeout, canlib.exceptions.CanNoMsg):
                break

    def __enter_bootloader(self, id):
        try:
            # Restart target
            self.ch.writeWait(
                rover.set_action_mode(city=id, mode=rover.ActionMode.RESET),
                self.default_timeout_ms,
            )

            time.sleep(0.02)  # Wait 20 ms for restart

            # Send default letter. Send should succeed only if node is on the bus again.
            self.ch.writeWait(rover.default_letter(), self.default_timeout_ms)

            self.__assign_bootloader_envelopes(id)

            # Allow target to communicate
            self.ch.writeWait(
                rover.set_comm_mode(city=id, mode=rover.CommMode.COMMUNICATE),
                self.default_timeout_ms,
            )

            f = Frame(id_=ENTER_BOOTLOADER_ID, dlc=0, data=[])

            self.__send_bootloader_command(f)
        except (Exception, canlib.exceptions.CanlibException) as e:
            raise RuntimeError(f"entering bootloader failed: {e}") from e

    def __exit_bootloader(self):
        f = Frame(id_=EXIT_BOOTLOADER_ID, dlc=0, data=[])
        try:
            self.__send_bootloader_command(f)
        except (Exception, canlib.exceptions.CanlibException) as e:
            raise RuntimeError(f"exiting bootloader failed: {e}") from e

    def __flash_erase(self, bytes_to_erase):
        f = Frame(
            id_=FLASH_ERASE_ID,
            dlc=4,
            data=list(bytes_to_erase.to_bytes(4, "little")),
        )
        try:
            self.__send_bootloader_command(f, timeout=10_000)
        except (Exception, canlib.exceptions.CanlibException) as e:
            raise RuntimeError(f"erasing flash failed: {e}") from e

    def __flash_program(self, binary_data):
        try:
            self.__block_transfer(FLASH_PROGRAM_ID, binary_data)
        except RuntimeError as e:
            raise RuntimeError(f"flashing binary failed: {e}") from e
        except (Exception, canlib.exceptions.CanlibException):
            raise

    def __format_fs(self):
        f = Frame(id_=FORMAT_FS_ID, dlc=0, data=[])
        try:
            self.__send_bootloader_command(f, timeout=10_000)
        except (Exception, canlib.exceptions.CanlibException) as e:
            raise RuntimeError(f"formatting FS failed: {e}") from e

    def __write_config(self, config):
        try:
            binary = json.dumps(config, separators=(",", ":")).encode(encoding="ascii")
            self.__block_transfer(FLASH_CONFIG_ID, binary)
        except RuntimeError as e:
            raise RuntimeError(f"writing config failed: {e}") from e
        except (Exception, canlib.exceptions.CanlibException):
            raise

    # Set all nodes except target to silent
    def __single_comm_mode(self, id):
        self.ch.write(
            rover.set_comm_mode(city=rover.City.ALL_CITIES, mode=rover.CommMode.SILENT)
        )
        self.ch.writeWait(
            rover.set_comm_mode(city=id, mode=rover.CommMode.COMMUNICATE),
            self.default_timeout_ms,
        )

    def __restart_all(self):
        self.ch.writeWait(
            rover.set_action_mode(mode=rover.ActionMode.RESET),
            self.default_timeout_ms,
        )

    def __assign_bootloader_envelopes(self, node_id):
        for assignment in assignments:
            envelope = assignment[0]
            folder = assignment[1]
            self.ch.writeWait(
                rover.assign_envelope(node_id, envelope, folder),
                self.default_timeout_ms,
            )

    def __send_bootloader_command(self, frame, timeout=None):
        if not timeout:
            timeout = self.default_timeout_ms

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


class FlasherConfig:
    def __init__(self, flasher_conf, binary_dir):

        try:
            f = _read_binary(flasher_conf)
            self.json = json.loads(f)
        except Exception:
            raise

        for name, val in self.json.items():
            if "id" not in val:
                raise ValueError(
                    f'invalid flasher configuration, {name} is missing "id" key'
                )

            if "binary" not in val:
                raise ValueError(
                    f'invalid flasher configuration, {name} is missing "binary" key'
                )

            if "config" not in val:
                raise ValueError(
                    f'invalid flasher configuration, {name} is missing "config" key'
                )

        self.node_binary_map = {}
        try:
            for node in self.json:
                file_path = Path(binary_dir, self.json[node]["binary"])
                self.node_binary_map[node] = _read_binary(file_path)

        except Exception:
            raise

    def get_nodes(self):
        return self.json.keys()

    def get_id(self, node):
        return self.json[node]["id"]

    def get_config(self, node):
        return self.json[node]["config"]

    def get_binary(self, node):
        return self.node_binary_map[node]

    def __repr__(self):
        return str(self.json)


def _read_binary(file):
    try:
        with open(Path(file), "rb") as f:
            return f.read()
    except Exception as e:
        raise ValueError(f"couldn't read {file}: {e}") from e


def _read_json(file):
    try:
        with open(Path(file), "r") as f:
            return json.load(f)
    except Exception as e:
        raise ValueError(f"couldn't read {file}: {e}") from e
