import keyboard
import time
import threading
import tkinter as tk
from tkinter import font
from canlib import canlib, kvadblib
from rover import Rover
import servo

unique_messages = {}
unique_messages_lock = threading.Lock()


def parse_frame(db, frame):
    output = ""
    try:
        bmsg = db.interpret(frame)
    except kvadblib.KvdNoMessage:
        print(f"<<< No message found for frame with id {frame.id} >>>")
        return ""

    if not bmsg._message.dlc == bmsg._frame.dlc:
        print(
            "<<< Could not interpret message because DLC does not match for"
            f" frame with id {frame.id} >>>"
        )
        print(f"\t- DLC (database): {bmsg._message.dlc}")
        print(f"\t- DLC (received frame): {bmsg._frame.dlc}")
        return ""

    msg = bmsg._message

    output = output + f"┏ Timestamp: {str(frame.timestamp/1000.0)}\n"
    output = output + f"┃ ID: {frame.id}, {msg.name}\n"

    # The steering and throttle messages have a signal which indicates how to
    # parse another signal. In that case, we should not output both
    # interpretations of the signal, hence we choose one signal and skip the
    # other in those messages.
    mux_signal = False
    chosen_signal = 0

    for n, bsig in enumerate(bmsg):
        if mux_signal and n != chosen_signal:
            continue

        output = output + f"┃ --> {bsig.name}: {str(bsig.value)} {bsig.unit}\n"

        if bsig.is_enum:
            mux_signal = True
            chosen_signal = bsig.raw + 1

    return output + "┗\n"


def receive_can_messages():
    db = kvadblib.Dbc(filename="rover.dbc")

    with canlib.openChannel(
        channel=0,
        flags=canlib.Open.NO_INIT_ACCESS,
    ) as ch:
        ch.busOn()
        while True:
            try:
                frame = ch.read(timeout=1000)
                if frame:
                    unique_messages_lock.acquire()
                    unique_messages[str(frame.id)] = parse_frame(db, frame)
                    unique_messages_lock.release()

            except KeyboardInterrupt:
                break


def update_ui():
    while True:
        try:
            received_text.config(state=tk.NORMAL)
            received_text.delete("1.0", tk.END)  # Clear existing text

            unique_messages_lock.acquire()

            for msg in unique_messages.values():
                received_text.insert(tk.END, msg)

            unique_messages_lock.release()

            received_text.config(state=tk.DISABLED)

            time.sleep(0.2)

        except KeyboardInterrupt:
            break


def send_can_messages():
    throttle = 1500
    steering = 1500

    throttle_max = 2000
    throttle_min = 1000

    steering_max = 2000
    steering_min = 1000

    step = 10

    # Initialize CAN channel
    with canlib.openChannel(
        channel=0,
        flags=canlib.Open.REQUIRE_INIT_ACCESS,
        bitrate=canlib.Bitrate.BITRATE_125K,
    ) as ch:
        ch.setBusOutputControl(canlib.Driver.NORMAL)
        ch.busOn()

        rover = Rover(ch)
        rover.start()

        while True:
            try:
                if keyboard.is_pressed("up"):
                    if throttle < throttle_max:
                        throttle += step

                elif keyboard.is_pressed("down"):
                    if throttle > throttle_min:
                        throttle -= step

                elif keyboard.is_pressed("left"):
                    if steering > steering_min:
                        steering -= step

                elif keyboard.is_pressed("right"):
                    if steering < steering_max:
                        steering += step

                ch.writeWait(servo.set_throttle_pulse_frame(throttle), -1)
                ch.writeWait(servo.set_steering_pulse_frame(steering), -1)
                time.sleep(0.01)

            except KeyboardInterrupt:
                break


# Create and configure GUI
root = tk.Tk()
root.title("CAN Message Viewer")
received_text = tk.Text(root, height=30, width=80)
received_text.pack()

# Set font size
font_size = 12  # Adjust the font size as needed
font_obj = font.Font(family="Consolas", size=font_size)
received_text.configure(font=font_obj)

# Create threads for sending and receiving messages
receive_thread = threading.Thread(target=receive_can_messages, daemon=True)
update_thread = threading.Thread(target=update_ui, daemon=True)
send_thread = threading.Thread(target=send_can_messages, daemon=True)

receive_thread.start()
update_thread.start()
send_thread.start()

root.mainloop()
