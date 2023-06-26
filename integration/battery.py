# Battery board testing frames

from canlib import Frame

# Assign envelope 0x100 to folder 2
assign_folder2 = Frame(id_=0, dlc=8, data=[0, 2, 0, 1, 0, 0, 2, 0x3])

# Assign envelope 0x101 to folder 3
assign_folder3 = Frame(id_=0, dlc=8, data=[0, 2, 1, 1, 0, 0, 3, 0x3])

# Assign envelope 0x102 to folder 4
assign_folder4 = Frame(id_=0, dlc=8, data=[0, 2, 2, 1, 0, 0, 4, 0x3])

# Assign envelope 0x103 to folder 5
assign_folder5 = Frame(id_=0, dlc=8, data=[0, 2, 3, 1, 0, 0, 5, 0x3])

# Assign envelope 0x105 to folder 7
assign_folder7 = Frame(id_=0, dlc=8, data=[0, 2, 5, 1, 0, 0, 7, 0x3])

# Assign envelope 0x107 to folder 9
assign_folder9 = Frame(id_=0, dlc=8, data=[0, 2, 7, 1, 0, 0, 9, 0x3])

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
