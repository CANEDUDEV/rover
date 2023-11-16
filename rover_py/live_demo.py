import collections
import os
import sys
import threading
import time
import tkinter as tk
from tkinter import font

import keyboard
import matplotlib.pyplot as plt
from canlib import canlib, kvadblib
from matplotlib.animation import FuncAnimation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from rover import servo
from rover.rover import Envelope, Rover

sent_messages = {}
received_messages = {}
lock = threading.Lock()

restart_rover = False

# Graphs
max_items_sent = 400
max_items_received = 100
servo_current_timestamps = collections.deque(maxlen=max_items_received)
servo_current_values = collections.deque(maxlen=max_items_received)
steering_timestamps = collections.deque(maxlen=max_items_sent)
steering_values = collections.deque(maxlen=max_items_sent)
throttle_timestamps = collections.deque(maxlen=max_items_sent)
throttle_values = collections.deque(maxlen=max_items_sent)


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

                with lock:
                    if frame.id == Envelope.THROTTLE:
                        sent_messages[str(frame.id)] = parse_frame(db, frame)
                        throttle_timestamps.append(frame.timestamp)
                        throttle_values.append(
                            int.from_bytes(frame.data[1:], byteorder="little")
                        )

                    elif frame.id == Envelope.STEERING:
                        sent_messages[str(frame.id)] = parse_frame(db, frame)
                        steering_timestamps.append(frame.timestamp)
                        steering_values.append(
                            int.from_bytes(frame.data[1:], byteorder="little")
                        )

                    elif frame.id == Envelope.SERVO_CURRENT:
                        received_messages[str(frame.id)] = parse_frame(db, frame)
                        servo_current_timestamps.append(frame.timestamp)
                        servo_current_values.append(
                            int.from_bytes(frame.data, byteorder="little")
                        )

                    else:
                        received_messages[str(frame.id)] = parse_frame(db, frame)

            except canlib.exceptions.CanNoMsg:
                continue


def update_ui():
    sent_messages_text.config(state=tk.NORMAL)
    received_messages_text.config(state=tk.NORMAL)

    # Clear existing text
    sent_messages_text.delete("1.0", tk.END)
    received_messages_text.delete("1.0", tk.END)

    with lock:
        for msg in sent_messages.values():
            sent_messages_text.insert(tk.END, msg)

        for msg in received_messages.values():
            received_messages_text.insert(tk.END, msg)

    sent_messages_text.config(state=tk.DISABLED)
    received_messages_text.config(state=tk.DISABLED)

    root.after(100, update_ui)


def send_can_messages():
    throttle = 1500
    steering = 1500

    throttle_max = 2000
    throttle_min = 1000

    steering_max = 2000
    steering_min = 1000

    step = 10

    global restart_rover

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
            if restart_rover:
                rover.start()
                restart_rover = False

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


def animate(_):
    ax1, ax2, ax3 = fig.get_axes()

    # Clear current data
    ax1.cla()
    ax2.cla()
    ax3.cla()

    ax1.set_title("Servo Current Draw (mA)")
    ax1.set_ylim(0, 250)

    ax2.set_title("Throttle Pulse Width (µs)")
    ax2.set_ylim(950, 2050)

    ax3.set_title("Steering Pulse Width (µs)")
    ax3.set_ylim(950, 2050)

    # Plot new data
    with lock:
        ax1.plot(servo_current_timestamps, servo_current_values)
        ax2.plot(throttle_timestamps, throttle_values)
        ax3.plot(steering_timestamps, steering_values)


def on_closing():
    root.destroy()
    sys.exit(0)


def restart_button_click():
    global restart_rover
    restart_rover = True


def zoom_in():
    global font_size
    font_size += 2
    set_font_size(font_size)


def zoom_out():
    global font_size
    font_size -= 2
    set_font_size(font_size)


def set_font_size(size):
    font_obj = font.Font(size=size + 4)

    sent_messages_text.configure(font=font_obj)
    sent_messages_label.configure(font=font_obj)
    received_messages_text.configure(font=font_obj)
    received_messages_label.configure(font=font_obj)
    restart_button.configure(font=font_obj)
    zoom_in_button.configure(font=font_obj)
    zoom_out_button.configure(font=font_obj)
    plt.rcParams.update({"font.size": font_size})


# Create and configure GUI
root = tk.Tk()
root.title("CAN Message Viewer")
root.protocol("WM_DELETE_WINDOW", on_closing)

root.state("zoomed")

can_messages_frame = tk.Frame(root, height=20)
can_messages_frame.pack(side="top", fill="both", expand=1, padx="0.5cm", pady="1cm")
sent_messages_frame = tk.Frame(can_messages_frame)
received_messages_frame = tk.Frame(can_messages_frame)
sent_messages_frame.pack(side="left", fill="x", expand=1, padx="0.5cm")
received_messages_frame.pack(side="left", fill="x", expand=1, padx="0.5cm")

sent_messages_label = tk.Label(sent_messages_frame, text="Sent messages")
sent_messages_text = tk.Text(sent_messages_frame, height=10, width=10)
sent_messages_label.pack(side="top", expand=1, anchor="w")
sent_messages_text.pack(side="top", fill="both", expand=1)

received_messages_label = tk.Label(received_messages_frame, text="Received messages")
received_messages_text = tk.Text(received_messages_frame, height=10, width=10)
received_messages_label.pack(side="top", expand=1, anchor="w")
received_messages_text.pack(side="top", fill="both", expand=1)

# Plotting
fig = plt.figure()
subplots = fig.subplots(1, 3)
fig.subplots_adjust(left=0.05, right=0.95, wspace=0.2)

ani = FuncAnimation(fig, animate, interval=100, blit=False, save_count=3000)

canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().pack(
    side="top", padx="1cm", expand=1, fill="both", anchor="center"
)

restart_button = tk.Button(root, text="Restart", command=restart_button_click)
restart_button.pack(side="right", pady=10, padx=40, anchor="e")

zoom_in_button = tk.Button(root, text="Zoom in", command=zoom_in)
zoom_in_button.pack(side="right", pady=10, padx=10, anchor="e")

zoom_out_button = tk.Button(root, text="Zoom out", command=zoom_out)
zoom_out_button.pack(side="right", pady=10, padx=10, anchor="e")

# Set font size
font_size = 14

# Create threads for sending and receiving messages
receive_thread = threading.Thread(target=receive_can_messages, daemon=True)
update_thread = threading.Thread(target=update_ui, daemon=True)
send_thread = threading.Thread(target=send_can_messages, daemon=True)

receive_thread.start()
update_thread.start()
send_thread.start()

root.mainloop()
