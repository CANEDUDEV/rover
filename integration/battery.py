# Battery board testing frames

from canlib import Frame
from rover import Envelope


def set_over_current_threshold_frame(current_ma):
    data = [0xFF, 0xFF, 0x01] + list(current_ma.to_bytes(4, "little"))
    return Frame(id_=Envelope.BATTERY_JUMPER_AND_FUSE_CONF, dlc=7, data=data)


def set_reg_out_voltage_frame(voltage_mv):
    data = list(voltage_mv.to_bytes(2, "little"))
    return Frame(id_=Envelope.BATTERY_REG_OUT_VOLTAGE, dlc=2, data=data)


set_reg_pwr_off_frame = Frame(
    id_=Envelope.BATTERY_OUTPUT_ON_OFF, dlc=2, data=[0, 1]
)  # Set regulated power out OFF

set_reg_pwr_on_frame = Frame(
    id_=Envelope.BATTERY_OUTPUT_ON_OFF, dlc=2, data=[1, 1]
)  # Set regulated power out ON

set_pwr_off_frame = Frame(
    id_=Envelope.BATTERY_OUTPUT_ON_OFF, dlc=2, data=[0, 0]
)  # Set power out OFF

set_pwr_on_frame = Frame(
    id_=Envelope.BATTERY_OUTPUT_ON_OFF, dlc=2, data=[1, 1]
)  # Set power out ON


def set_low_voltage_cutoff_frame(voltage_mv):
    data = list(voltage_mv.to_bytes(2, "little"))
    return Frame(id_=Envelope.BATTERY_LOW_VOLTAGE_CUTOFF, dlc=2, data=data)


def set_monitor_period_frame(time_ms):
    data = list(time_ms.to_bytes(2, "little")) + [0, 0]
    return Frame(id_=Envelope.BATTERY_REPORT_FREQUENCY, dlc=4, data=data)


def set_report_period_frame(time_ms):
    data = [0, 0] + list(time_ms.to_bytes(2, "little"))
    return Frame(id_=Envelope.BATTERY_REPORT_FREQUENCY, dlc=4, data=data)
