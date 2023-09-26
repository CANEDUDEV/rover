import sys
from canlib import canlib, Frame

# TODO: only address node to be flashed.
BOOTLOADER_ADDRESS = 0  # Address all nodes

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


default_letter = Frame(id_=2031, data=[0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA])

# Give base number (0x400) and ask for response page 2
# All nodes currently in the bootloader will respond,
# since page 2 is only present in the bootloader.
give_base_number = Frame(id_=0, dlc=8, data=[0, 1, 2, 0, 0, 4, 0, 0])

# Set communication mode to COMMUNICATE
communicate = Frame(id_=0, dlc=8, data=[BOOTLOADER_ADDRESS, 0, 0, 0x3, 0, 0, 0, 0])

enter_bootloader = Frame(id_=ENTER_BOOTLOADER_ID, dlc=0, data=[])

# Set bitrate to 1Mbit/s
change_bitrate_1mbit = Frame(id_=0, dlc=8, data=[0, 8, 0, 0, 4, 9, 1, 1])

# Reset communication. All nodes keep their current communication mode.
# Startup sequence and wait for default letter are skipped.
reset = Frame(id_=0, dlc=8, data=[0, 0, 0, 0x2C, 0, 0, 0, 0])


def flash_erase(bytes_to_erase):
    return Frame(
        id_=FLASH_ERASE_ID,
        dlc=4,
        data=list(bytes_to_erase.to_bytes(4, "little")),
    )


def flash_program(ch, binary_data, timeout=100):
    # init
    data = [1] + list(len(binary_data).to_bytes(4, "little")) + [0, 0, 0]
    ch.writeWait(Frame(id_=FLASH_PROGRAM_ID, dlc=8, data=data), -1)

    try:
        received = ch.read(timeout=timeout)
    except canlib.exceptions.CanNoMsg:
        print("No bundle request response (1). Exiting.")
        sys.exit(1)

    if received.id != FLASH_PROGRAM_ID or received.data[0] != 2:
        print("Wrong response before programming flash.")
        sys.exit(1)

    if int.from_bytes(received.data[1:3], byteorder="little") != 0xFFFF:
        print("Got wrong bundle request, wanted 0xFFFF.")
        sys.exit(1)

    # Send pages with data.
    chunk_size = 7  # Bytes per frame

    # Get bytes that need to be padded
    final_bytes = list(binary_data[-(len(binary_data) % chunk_size) :])

    current_page_number = 3

    # Send bytes in chunks of 7
    for i in range(0, len(binary_data) - len(final_bytes), chunk_size):
        chunk = list(binary_data[i : i + chunk_size])
        data = [current_page_number] + chunk
        ch.writeWait(Frame(id_=FLASH_PROGRAM_ID, dlc=8, data=data), -1)

        if current_page_number == 3:
            current_page_number = 4
        else:
            current_page_number = 3

    final_data = [current_page_number] + final_bytes

    # Pad if needed
    for i in range(8 - len(final_data)):
        final_data.append(0)

    ch.writeWait(Frame(id_=FLASH_PROGRAM_ID, dlc=8, data=final_data), -1)

    try:
        received = ch.read(timeout=timeout)
    except canlib.exceptions.CanNoMsg:
        print("No bundle request response (2). Exiting.")
        sys.exit(1)

    if received.id != FLASH_PROGRAM_ID or received.data[0] != 2:
        print("Wrong response after programming flash.")
        sys.exit(1)

    if int.from_bytes(received.data[1:3], byteorder="little") != 0x0000:
        print("Got wrong bundle request, wanted 0x0000")
        sys.exit(1)


exit_bootloader = Frame(id_=EXIT_BOOTLOADER_ID, dlc=0, data=[])


def send_bootloader_command(ch, frame, timeout=100):
    ch.writeWait(frame, -1)
    try:
        received = ch.read(timeout=timeout)

    except canlib.exceptions.CanNoMsg:
        print("No response to command. Exiting.")
        sys.exit(1)

    if received.id != COMMAND_ACK_ID:
        print("Not an ACK message")
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


# Get filename
if len(sys.argv) < 2:
    print(f"Usage: python {sys.argv[0]} <app.bin>")
    sys.exit(1)

# Open file and store contents in binary_data
with open(sys.argv[1], "rb") as file:
    binary_data = file.read()


with canlib.openChannel(
    channel=0,
    flags=canlib.Open.REQUIRE_INIT_ACCESS,
    bitrate=canlib.Bitrate.BITRATE_125K,
) as ch:
    ch.setBusOutputControl(canlib.Driver.NORMAL)
    ch.busOn()

    ch.writeWait(default_letter, -1)
    ch.writeWait(give_base_number, -1)

    while True:
        frame = ch.read(timeout=1000)
        if frame and not (frame.flags & canlib.MessageFlag.ERROR_FRAME):
            break

    for assignment in assignments:
        envelope = list(assignment[0].to_bytes(4, "little"))
        folder = assignment[1]
        data = [BOOTLOADER_ADDRESS, 2] + envelope + [folder] + [0x3]
        ch.writeWait(Frame(id_=0, dlc=8, data=data), -1)

    # TODO: Set all nodes to Silent mode, except node to be flashed
    ch.writeWait(communicate, -1)

    send_bootloader_command(ch, enter_bootloader)

    ch.writeWait(change_bitrate_1mbit, -1)
    ch.writeWait(reset, -1)

    ch.busOff()

# Switch to 1 Mbit/s for flashing
with canlib.openChannel(
    channel=0,
    flags=canlib.Open.REQUIRE_INIT_ACCESS,
    bitrate=canlib.Bitrate.BITRATE_1M,
) as ch:
    ch.setBusOutputControl(canlib.Driver.NORMAL)
    ch.busOn()

    # Erase as many pages as required to fit the application
    send_bootloader_command(ch, flash_erase(len(binary_data)), timeout=10_000)

    flash_program(ch, binary_data, timeout=1000)

    send_bootloader_command(ch, exit_bootloader)

    ch.busOff()
