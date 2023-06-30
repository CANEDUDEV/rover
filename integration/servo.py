# Servo board testing frames

from canlib import Frame

# Assign envelope 0x200 to folder 2
assign_folder2 = Frame(id_=0, dlc=8, data=[0, 2, 0, 2, 0, 0, 2, 0x3])

# Assign envelope 0x201 to folder 3
assign_folder3 = Frame(id_=0, dlc=8, data=[0, 2, 1, 2, 0, 0, 3, 0x3])

# Assign envelope 0x202 to folder 4
assign_folder4 = Frame(id_=0, dlc=8, data=[0, 2, 2, 2, 0, 0, 4, 0x3])

# Assign envelope 0x203 to folder 5
assign_folder5 = Frame(id_=0, dlc=8, data=[0, 2, 3, 2, 0, 0, 5, 0x3])

# Assign envelope 0x204 to folder 6
assign_folder6 = Frame(id_=0, dlc=8, data=[0, 2, 4, 2, 0, 0, 6, 0x3])
