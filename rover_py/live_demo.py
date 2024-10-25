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

from rover import ActionMode, City, Envelope, rover, servo

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
    # we need to look for the file in sys._MEIPASS, which is set by pyinstaller.
    # Otherwise, just look in the current dir.
    try:
        base_path = sys._MEIPASS  # pyright: ignore [reportAttributeAccessIssue]
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

        ch.write(servo.set_failsafe(servo.FAILSAFE_OFF, city=City.SERVO))
        ch.write(servo.set_failsafe(servo.FAILSAFE_OFF, city=City.MOTOR))
        ch.write(rover.set_action_mode(city=City.SBUS_RECEIVER, mode=ActionMode.FREEZE))

        while True:
            if restart_rover:
                ch.write(servo.set_failsafe(servo.FAILSAFE_OFF, city=City.SERVO))
                ch.write(servo.set_failsafe(servo.FAILSAFE_OFF, city=City.MOTOR))
                ch.write(
                    rover.set_action_mode(
                        city=City.SBUS_RECEIVER, mode=ActionMode.FREEZE
                    )
                )
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

            ch.write(servo.set_throttle_pulse_frame(throttle))
            ch.write(servo.set_steering_pulse_frame(steering))
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
    font_obj = font.Font(size=size)

    sent_messages_text.configure(font=font_obj)
    sent_messages_label.configure(font=font_obj)
    received_messages_text.configure(font=font_obj)
    received_messages_label.configure(font=font_obj)
    restart_button.configure(font=font_obj)
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
subplots = fig.subplots(1, 3)
fig.subplots_adjust(left=0.05, right=0.95, wspace=0.2)

ani = FuncAnimation(
    fig,
    animate,  # pyright: ignore [reportArgumentType]
    interval=100,
    blit=False,
    save_count=3000,
)

canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().pack(
    side="top", padx=20, expand=True, fill="both", anchor="center"
)

button_frame = tk.Frame(root)
button_frame.pack(side="top", fill="both", expand=True)

restart_button = tk.Button(button_frame, text="Restart", command=restart_button_click)
restart_button.pack(side="right", pady=10, padx=40, anchor="e")

zoom_in_button = tk.Button(button_frame, text="Zoom in", command=zoom_in)
zoom_in_button.pack(side="right", pady=10, padx=10, anchor="e")

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
