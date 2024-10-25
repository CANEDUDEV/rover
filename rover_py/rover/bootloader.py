import enum

from . import Assignment, Envelope


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


def generate_assignments(id):
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
