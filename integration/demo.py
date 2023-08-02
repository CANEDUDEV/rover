from canlib import canlib
import matplotlib.pyplot as plt
from collections import deque

import rover
import servo

# Set up Matplotlib with subplots for both CAN signals
plt.ion()
fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(10, 8))

# Set up subplot for the first CAN signal (Steering signal)
ax1.set_ylabel("Pulse width (µs)")
ax1.set_title("Steering Signal")
ax1.grid(False)

# Set up subplot for the second CAN signal (Servo current signal)
ax2.set_xlabel("Time since start (s)")
ax2.set_ylabel("Current (mA)")
ax2.set_title("Servo Current Draw")
ax2.grid(False)

# Set up data storage for both signals
max_data_points = 2000
time_data1 = deque(maxlen=max_data_points)
value_data1 = deque(maxlen=max_data_points)
time_data2 = deque(maxlen=max_data_points)
value_data2 = deque(maxlen=max_data_points)

# Initialize CANlib for both signals
with canlib.openChannel(
    channel=0, flags=canlib.Open.REQUIRE_INIT_ACCESS, bitrate=canlib.canBITRATE_125K
) as ch:
    ch.setBusOutputControl(canlib.Driver.NORMAL)
    ch.busOn()

    # Set up data batching to make plotting more responsive.
    batch_time_data1 = []
    batch_value_data1 = []
    batch_time_data2 = []
    batch_value_data2 = []

    # Adjust the servo monitoring and report frequency to pick up current spikes.
    ch.writeWait(servo.set_measure_period_frame(20), -1)
    ch.writeWait(servo.set_report_period_frame(20), -1)

    while True:
        try:
            frame = ch.read(timeout=-1)
            if frame is None:
                continue

            timestamp = frame.timestamp / 1000

            # Process the first CAN signal (Steering signal)
            if frame.id == rover.Envelope.STEERING:
                can_value = int.from_bytes(bytes(frame.data[1:]), byteorder="little")
                batch_time_data1.append(timestamp)
                batch_value_data1.append(can_value)

                if len(batch_time_data1) >= 10:
                    time_data1.extend(batch_time_data1)
                    value_data1.extend(batch_value_data1)
                    batch_time_data1.clear()
                    batch_value_data1.clear()

                    # Update and redraw the first subplot
                    ax1.clear()
                    ax1.plot(time_data1, value_data1)
                    ax1.set_ylim(1000, 2000)
                    ax1.set_ylabel("Pulse width (µs)")
                    ax1.set_title("Steering Signal")
                    ax1.grid(False)
                    ax1.set_xlim(timestamp - 5, timestamp)

                    plt.pause(0.01)

            # Process the second CAN signal (Servo current)
            elif frame.id == rover.Envelope.SERVO_CURRENT:
                can_value = int.from_bytes(bytes(frame.data[0:]), byteorder="little")
                batch_time_data2.append(timestamp)
                batch_value_data2.append(can_value)

                if len(batch_time_data2) >= 10:
                    time_data2.extend(batch_time_data2)
                    value_data2.extend(batch_value_data2)
                    batch_time_data2.clear()
                    batch_value_data2.clear()

                    # Update and redraw the second subplot
                    ax2.clear()
                    ax2.plot(time_data2, value_data2)
                    ax2.set_ylim(0, 5000)
                    ax2.set_xlabel("Time since start (s)")
                    ax2.set_ylabel("Current (mA)")
                    ax2.set_title("Servo Current Draw")
                    ax2.grid(False)
                    ax2.set_xlim(timestamp - 5, timestamp)

                    plt.pause(0.01)

        except KeyboardInterrupt:
            break

        except Exception as e:
            print("Error:", e)
            break

        # Check if the plot window is closed by the user
        if not plt.fignum_exists(fig.number):
            break


# Clean up
plt.ioff()
plt.show()
