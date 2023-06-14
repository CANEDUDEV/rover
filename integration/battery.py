# Battery board testing frames

from canlib import Frame

assign_folder2 = Frame(
    id_=0, dlc=8, data=[0, 2, 0, 1, 0, 0, 2, 0x3]
)  # Assign envelope 0x100 to folder 2

assign_folder3 = Frame(
    id_=0, dlc=8, data=[0, 2, 1, 1, 0, 0, 3, 0x3]
)  # Assign envelope 0x101 to folder 3

assign_folder4 = Frame(
    id_=0, dlc=8, data=[0, 2, 2, 1, 0, 0, 4, 0x3]
)  # Assign envelope 0x102 to folder 4
