# SBUS receiver testing frames

from canlib import Frame
import ck

# Assign envelope 0x207 to folder 2
# Assign steering pulse width to servo's address
assign_steering_tx = Frame(
    id_=0, dlc=8, data=[ck.sbus_receiver_id, 2, 7, 2, 0, 0, 2, 0x3]
)

# Assign envelope 0x208 to folder 3
# Steering trim pulse width (signed integer)
assign_steering_trim_tx = Frame(
    id_=0, dlc=8, data=[ck.sbus_receiver_id, 2, 8, 2, 0, 0, 3, 0x3]
)

# Assign envelope 0x302 to folder 4
# Throttle pulse width
assign_throttle_tx = Frame(
    id_=0, dlc=8, data=[ck.sbus_receiver_id, 2, 2, 3, 0, 0, 4, 0x3]
)

# Assign envelope 0x303 to folder 5
# Throttle trim pulse width (signed integer)
assign_throttle_trim_tx = Frame(
    id_=0, dlc=8, data=[ck.sbus_receiver_id, 2, 3, 3, 0, 0, 5, 0x3]
)
