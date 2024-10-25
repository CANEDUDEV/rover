import struct

from canlib import Frame

from . import City, Envelope


def set_wheel_parameters_frame(
    cog_count, wheel_diameter_m, wheel_id=City.WHEEL_FRONT_LEFT
):
    envelope = Envelope.WHEEL_FRONT_LEFT_WHEEL_PARAMETERS
    if wheel_id == City.WHEEL_FRONT_RIGHT:
        envelope = Envelope.WHEEL_FRONT_RIGHT_WHEEL_PARAMETERS
    if wheel_id == City.WHEEL_REAR_LEFT:
        envelope = Envelope.WHEEL_REAR_LEFT_WHEEL_PARAMETERS
    if wheel_id == City.WHEEL_REAR_RIGHT:
        envelope = Envelope.WHEEL_REAR_RIGHT_WHEEL_PARAMETERS

    data = list(cog_count.to_bytes(4, "little")) + list(
        struct.pack("f", wheel_diameter_m)
    )

    return Frame(id_=envelope, dlc=8, data=data)


def set_report_period_frame(time_ms, wheel_id=City.WHEEL_FRONT_LEFT):
    envelope = Envelope.WHEEL_FRONT_LEFT_REPORT_FREQUENCY
    if wheel_id == City.WHEEL_FRONT_RIGHT:
        envelope = Envelope.WHEEL_FRONT_RIGHT_REPORT_FREQUENCY
    if wheel_id == City.WHEEL_REAR_LEFT:
        envelope = Envelope.WHEEL_REAR_LEFT_REPORT_FREQUENCY
    if wheel_id == City.WHEEL_REAR_RIGHT:
        envelope = Envelope.WHEEL_REAR_RIGHT_REPORT_FREQUENCY

    data = list(time_ms.to_bytes(2, "little"))

    return Frame(id_=envelope, dlc=2, data=data)
