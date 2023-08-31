import keyboard
import os
import sys
import time
import threading
import tkinter as tk
from tkinter import font
from canlib import canlib, kvadblib
from rover import Envelope, Rover
import servo


sent_messages = {}
received_messages = {}
messages_lock = threading.Lock()


def parse_frame(db, frame):
    output = ""
    try:
        bmsg = db.interpret(frame)
    except kvadblib.KvdNoMessage:
        return ""

    if not bmsg._message.dlc == bmsg._frame.dlc:
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
    # If packaged as one binary using pyinstaller,
    # we need to look for the file in sys._MEIPASS.
    # Otherwise, just look in the current dir.
    try:
        base_path = sys._MEIPASS
    except Exception:
        base_path = os.path.abspath(".")

    db = kvadblib.Dbc(filename=os.path.join(base_path, "rover.dbc"))

    with canlib.openChannel(
        channel=0,
        flags=canlib.Open.NO_INIT_ACCESS,
    ) as ch:
        ch.busOn()

        while True:
            try:
                frame = ch.read(timeout=1000)
                if not frame:
                    continue

                with messages_lock:
                    if frame.id == Envelope.THROTTLE or frame.id == Envelope.STEERING:
                        sent_messages[str(frame.id)] = parse_frame(db, frame)
                    else:
                        received_messages[str(frame.id)] = parse_frame(db, frame)

            except canlib.exceptions.CanNoMsg:
                continue

            except KeyboardInterrupt:
                break


def update_ui():
    while True:
        try:
            sent_messages_text.config(state=tk.NORMAL)
            received_messages_text.config(state=tk.NORMAL)

            # Clear existing text
            sent_messages_text.delete("1.0", tk.END)
            received_messages_text.delete("1.0", tk.END)

            with messages_lock:
                for msg in sent_messages.values():
                    sent_messages_text.insert(tk.END, msg)

                for msg in received_messages.values():
                    received_messages_text.insert(tk.END, msg)

            sent_messages_text.config(state=tk.DISABLED)
            received_messages_text.config(state=tk.DISABLED)

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

height = 10
width = 40

sent_messages_label = tk.Label(root, text="Sent messages")
sent_messages_text = tk.Text(root, height=height, width=width)
sent_messages_label.grid(row=0, column=0, columnspan=1)
sent_messages_text.grid(row=1, column=0)

received_messages_label = tk.Label(root, text="Received messages")
received_messages_text = tk.Text(root, height=height, width=width)
received_messages_label.grid(row=0, column=1, columnspan=2)
received_messages_text.grid(row=1, column=1)

# Set font size
font_size = 18  # Adjust the font size as needed
font_obj = font.Font(family="Consolas", size=font_size)

sent_messages_text.configure(font=font_obj)
sent_messages_label.configure(font=font_obj)
received_messages_text.configure(font=font_obj)
received_messages_label.configure(font=font_obj)

# Create threads for sending and receiving messages
receive_thread = threading.Thread(target=receive_can_messages, daemon=True)
update_thread = threading.Thread(target=update_ui, daemon=True)
send_thread = threading.Thread(target=send_can_messages, daemon=True)

receive_thread.start()
update_thread.start()
send_thread.start()

root.mainloop()
