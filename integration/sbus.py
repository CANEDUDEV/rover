# SBUS receiver testing frames

from canlib import Frame

# Assign envelope 0x300 to folder 2
assign_folder2 = Frame(id_=0, dlc=8, data=[0, 2, 0, 3, 0, 0, 2, 0x3])

# Assign envelope 0x301 to folder 3
assign_folder3 = Frame(id_=0, dlc=8, data=[0, 2, 1, 3, 0, 0, 3, 0x3])

# Assign envelope 0x302 to folder 4
assign_folder4 = Frame(id_=0, dlc=8, data=[0, 2, 2, 3, 0, 0, 4, 0x3])

# Assign envelope 0x303 to folder 5
assign_folder5 = Frame(id_=0, dlc=8, data=[0, 2, 3, 3, 0, 0, 5, 0x3])
