# Battery board testing frames

from canlib import Frame
import ck

# Assign envelope 0x100 to folder 2
# Cell voltages transmit
assign_folder2 = Frame(
    id_=0, dlc=8, data=[ck.battery_monitor_id, 2, 0, 1, 0, 0, 2, 0x3]
)

# Assign envelope 0x101 to folder 3
# Regulated output current transmit
assign_folder3 = Frame(
    id_=0, dlc=8, data=[ck.battery_monitor_id, 2, 1, 1, 0, 0, 3, 0x3]
)

# Assign envelope 0x102 to folder 4
# Unregulated output current transmit
assign_folder4 = Frame(
    id_=0, dlc=8, data=[ck.battery_monitor_id, 2, 2, 1, 0, 0, 4, 0x3]
)

# Assign envelope 0x103 to folder 5
# Jumper and fuse config, over-current threck.battery_monitor_idhold receive
assign_folder5 = Frame(
    id_=0, dlc=8, data=[ck.battery_monitor_id, 2, 3, 1, 0, 0, 5, 0x3]
)

# Assign envelope 0x104 to folder 6
# Regulated out voltage receive
assign_folder6 = Frame(
    id_=0, dlc=8, data=[ck.battery_monitor_id, 2, 4, 1, 0, 0, 6, 0x3]
)

# Assign envelope 0x105 to folder 7
# Output on/off receive
assign_folder7 = Frame(
    id_=0, dlc=8, data=[ck.battery_monitor_id, 2, 5, 1, 0, 0, 7, 0x3]
)

# Assign envelope 0x106 to folder 8
# CAN report frequency receive
assign_folder8 = Frame(
    id_=0, dlc=8, data=[ck.battery_monitor_id, 2, 6, 1, 0, 0, 8, 0x3]
)

# Assign envelope 0x107 to folder 9
# Low voltage cutoff value receive
assign_folder9 = Frame(
    id_=0, dlc=8, data=[ck.battery_monitor_id, 2, 7, 1, 0, 0, 9, 0x3]
)

# Set over-current threshold to 0mA. All other values are ignored.
set_0ma_current = Frame(id_=0x103, dlc=7, data=[0xFF, 0xFF, 0x01, 0, 0, 0, 0])

# Set over-current threshold to 49500mA. All other values are ignored.
set_49500ma_current = Frame(id_=0x103, dlc=7, data=[0xFF, 0xFF, 0x01, 0x5C, 0xC1, 0, 0])

set_reg_pwr_off = Frame(id_=0x105, dlc=2, data=[0, 1])  # Set regulated power out OFF

set_reg_pwr_on = Frame(id_=0x105, dlc=2, data=[1, 1])  # Set regulated power out ON

set_pwr_off = Frame(id_=0x105, dlc=2, data=[0, 0])  # Set power out OFF

set_pwr_on = Frame(id_=0x105, dlc=2, data=[1, 1])  # Set power out ON

# Set low-voltage cutoff value to 4200mV.
set_4200mv_cutoff = Frame(id_=0x107, dlc=2, data=[0x68, 0x10])

# Set low-voltage cutoff value to 3200mV.
set_3200mv_cutoff = Frame(id_=0x107, dlc=2, data=[0x80, 0xC])

# Set monitor frequency to 500 ms. Ignore report frequency.
set_monitor_freq_500ms = Frame(id_=0x106, dlc=4, data=[0xF4, 0x1, 0, 0])

# Set report frequency to 1000 ms. Ignore monitor frequency.
set_report_freq_1000ms = Frame(id_=0x106, dlc=4, data=[0, 0, 0xE8, 0x3])
