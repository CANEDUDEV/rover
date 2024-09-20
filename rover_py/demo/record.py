import csv  # Import the CSV module

from canlib import canlib

from ..rover import rover, servo

# Set up data storage for steering signal and servo current signals.
steering_timestamps = []
steering_data = []
current_timestamps = []
current_data = []

recording_length_s = 15

# 7ms interval between messages, we want to store 30 seconds of points.
max_steering_data_len = round(recording_length_s * 1000 / 7)
# 20ms interval between messages, we want to store 30 seconds of points.
max_current_data_len = round(recording_length_s * 1000 / 20)

# Initialize CANlib for both signals
with canlib.openChannel(
    channel=0, flags=canlib.Open.REQUIRE_INIT_ACCESS, bitrate=canlib.canBITRATE_125K
) as ch:
    ch.setBusOutputControl(canlib.Driver.NORMAL)
    ch.busOn()

    ch.writeWait(servo.set_report_period_frame(20), -1)

    try:
        while (
            len(steering_data) < max_steering_data_len
            and len(current_data) < max_current_data_len
        ):
            frame = ch.read(timeout=-1)
            if frame is None:
                continue

            if frame.timestamp is None:
                continue

            timestamp = frame.timestamp / 1000

            # Process the first CAN signal (Steering signal)
            if frame.id == rover.Envelope.STEERING:
                can_value = int.from_bytes(bytes(frame.data[1:]), byteorder="little")
                steering_timestamps.append(timestamp)
                steering_data.append(can_value)

            # Process the second CAN signal (Servo current)
            elif frame.id == rover.Envelope.SERVO_CURRENT:
                can_value = int.from_bytes(bytes(frame.data[0:]), byteorder="little")
                current_timestamps.append(timestamp)
                current_data.append(can_value)

    except KeyboardInterrupt:
        pass

    except Exception as e:
        print("Error:", e)

# Save collected data to CSV files
with open("steering_data.csv", "w", newline="") as steering_file:
    writer = csv.writer(steering_file)
    writer.writerow(["Timestamp", "Steering Value"])
    writer.writerows(zip(steering_timestamps, steering_data))

with open("current_data.csv", "w", newline="") as current_file:
    writer = csv.writer(current_file)
    writer.writerow(["Timestamp", "Current Value"])
    writer.writerows(zip(current_timestamps, current_data))
