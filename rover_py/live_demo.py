import collections
import os
import sys
import threading
import time
import tkinter as tk
from tkinter import font

import keyboard
import matplotlib.pyplot as plt
from canlib import Frame, canlib, kvadblib
from matplotlib.animation import FuncAnimation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

sent_messages = {}
received_messages = {}
lock = threading.Lock()

brake = False

# Graphs
max_items_sent = 400
max_items_received = 100
servo_current_timestamps = collections.deque(maxlen=max_items_received)
servo_current_values = collections.deque(maxlen=max_items_received)

servo_position_timestamps = collections.deque(maxlen=max_items_received)
servo_position_values = collections.deque(maxlen=max_items_received)


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
                    if frame.id == 0x104:
                        sent_messages[str(frame.id)] = parse_frame(db, frame)

                    elif frame.id == 0x105:
                        sent_messages[str(frame.id)] = parse_frame(db, frame)

                    elif frame.id == 0x205:
                        received_messages[str(frame.id)] = parse_frame(db, frame)
                        servo_current_timestamps.append(frame.timestamp)
                        servo_current_values.append(
                            int.from_bytes(frame.data, byteorder="little")
                        )

                    elif frame.id == 0x30D:
                        received_messages[str(frame.id)] = parse_frame(db, frame)
                        servo_position_timestamps.append(frame.timestamp)
                        servo_position_values.append(
                            int.from_bytes(frame.data, byteorder="little")
                        )

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

    # Initialize CAN channel
    with canlib.openChannel(
        channel=0,
        flags=canlib.Open.REQUIRE_INIT_ACCESS,
        bitrate=canlib.Bitrate.BITRATE_125K,
    ) as ch:
        ch.setBusOutputControl(canlib.Driver.NORMAL)
        ch.busOn()

        while True:
            if brake:
                ch.writeWait(Frame(id_=0x104, dlc=0, data=[]), -1)
            else:
                ch.writeWait(Frame(id_=0x105, dlc=0, data=[]), -1)

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

            # ch.writeWait(servo.set_throttle_pulse_frame(throttle), -1)
            # ch.writeWait(servo.set_steering_pulse_frame(steering), -1)
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
    ax1, ax2 = fig.get_axes()

    # Clear current data
    ax1.cla()
    ax2.cla()

    ax1.set_title("Servo Current Draw (mA)")
    ax1.set_ylim(0, 2000)

    ax2.set_title("Servo position (degrees)")
    ax2.set_ylim(0, 90)

    # Plot new data
    with lock:
        ax1.plot(servo_current_timestamps, servo_current_values)
        ax2.plot(servo_position_timestamps, servo_position_values)


def on_closing():
    root.destroy()
    sys.exit(0)


def brake_button_click():
    global brake
    brake = True


def release_brake_button_click():
    global brake
    brake = False


def zoom_in():
    global font_size
    font_size += 2
    set_font_size(font_size)


def zoom_out():
    global font_size
    font_size -= 2
    set_font_size(font_size)


def set_font_size(size):
    font_obj = font.Font(size=size)

    sent_messages_text.configure(font=font_obj)
    sent_messages_label.configure(font=font_obj)
    received_messages_text.configure(font=font_obj)
    received_messages_label.configure(font=font_obj)
    brake_button.configure(font=font_obj)
    release_brake_button.configure(font=font_obj)
    zoom_in_button.configure(font=font_obj)
    zoom_out_button.configure(font=font_obj)
    plt.rcParams.update({"font.size": font_size})


def get_screen_dpi():
    width_pixels = root.winfo_fpixels("1i")
    height_pixels = root.winfo_fpixels("1i")
    screen_width_pixels = root.winfo_screenwidth()
    screen_height_pixels = root.winfo_screenheight()

    dpi_width = screen_width_pixels / width_pixels
    dpi_height = screen_height_pixels / height_pixels

    return dpi_width, dpi_height


# Create and configure GUI
root = tk.Tk()
root.title("CAN Message Viewer")
root.protocol("WM_DELETE_WINDOW", on_closing)
root.state("zoomed")

dpi_width, dpi_height = get_screen_dpi()

can_messages_frame = tk.Frame(root)
can_messages_frame.pack(side="top", fill="both", expand=True, padx=10, pady=5)
sent_messages_frame = tk.Frame(can_messages_frame)
received_messages_frame = tk.Frame(can_messages_frame)
sent_messages_frame.pack(side="left", fill="x", expand=True, padx=5)
received_messages_frame.pack(side="left", fill="x", expand=True, padx=5)

sent_messages_label = tk.Label(sent_messages_frame, text="Sent messages")
sent_messages_text = tk.Text(sent_messages_frame, height=12, width=10)
sent_messages_label.pack(side="top", expand=True, anchor="w")
sent_messages_text.pack(side="top", fill="both", expand=True)

received_messages_label = tk.Label(received_messages_frame, text="Received messages")
received_messages_text = tk.Text(received_messages_frame, height=12, width=10)
received_messages_label.pack(side="top", expand=True, anchor="w")
received_messages_text.pack(side="top", fill="both", expand=True)

# Plotting
fig = plt.figure()
subplots = fig.subplots(1, 2)
fig.subplots_adjust(left=0.05, right=0.95, wspace=0.2)

ani = FuncAnimation(fig, animate, interval=100, blit=False, save_count=3000)

canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().pack(
    side="top", padx=20, expand=True, fill="both", anchor="center"
)

button_frame = tk.Frame(root)
button_frame.pack(side="top", fill="both", expand=True)

brake_button = tk.Button(button_frame, text="Brake", command=brake_button_click)
brake_button.pack(side="right", pady=10, padx=40, anchor="e")

release_brake_button = tk.Button(
    button_frame, text="Release brake", command=release_brake_button_click
)
release_brake_button.pack(side="right", pady=10, padx=10, anchor="e")

zoom_in_button = tk.Button(button_frame, text="Zoom in", command=zoom_in)
zoom_in_button.pack(side="right", pady=10, padx=40, anchor="e")

zoom_out_button = tk.Button(button_frame, text="Zoom out", command=zoom_out)
zoom_out_button.pack(side="right", pady=10, padx=10, anchor="e")

# Set font size
font_size = int(round(max(dpi_width, dpi_height) * 1))
print(dpi_width, dpi_height)
print(font_size)
set_font_size(font_size)

# Create threads for sending and receiving messages
receive_thread = threading.Thread(target=receive_can_messages, daemon=True)
update_thread = threading.Thread(target=update_ui, daemon=True)
send_thread = threading.Thread(target=send_can_messages, daemon=True)

receive_thread.start()
update_thread.start()
send_thread.start()

root.mainloop()
