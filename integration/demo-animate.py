import matplotlib.pyplot as plt
import matplotlib.animation as animation
import csv


# Load data from CSV files
def load_data(file_name):
    timestamps = []
    values = []
    with open(file_name, "r") as file:
        reader = csv.reader(file)
        next(reader)  # Skip header
        for row in reader:
            timestamps.append(float(row[0]))
            values.append(int(row[1]))
    return timestamps, values


# Load data
steering_timestamps, steering_data = load_data("steering_data.csv")
current_timestamps, current_data = load_data("current_data.csv")


# Set up the plot
fig, ax = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
(line_steering,) = ax[0].plot([], [], label="Steering Value")
(line_current,) = ax[1].plot([], [], label="Current Value")

x_limit = 15  # Number of seconds to show initially
ax[0].set_title("Steering Signal")
ax[0].set_ylim(1000, 2000)
ax[0].set_ylabel("Pulse width (µs)")
ax[0].set_xlim(0, x_limit)
ax[0].grid(False)

ax[1].set_title("Servo Current Draw")
ax[1].set_ylim(0, 5000)
ax[1].set_ylabel("Current (mA)")
ax[1].set_xlim(0, x_limit)
ax[1].grid(False)

total_animation_length = 15  # Animation length in seconds

# Calculate the number of frames for animations (30 fps)
num_frames = int(total_animation_length * 60)
frame_interval_ms = 1000 / 60


# Update function for the animation
def update(num, timestamp, data, line):
    index = 0
    target = num * frame_interval_ms / 1000
    for count, value in enumerate(timestamp):
        if value >= target:
            index = count
            break

    line.set_data(timestamp[:index], data[:index])
    return (line,)


# Create a new animation that includes both subplots
def combined_update(num):
    update(num, steering_timestamps, steering_data, line_steering)
    update(num, current_timestamps, current_data, line_current)
    return line_steering, line_current


combined_ani = animation.FuncAnimation(
    fig,
    combined_update,
    frames=num_frames,
    blit=True,
    interval=frame_interval_ms,
    repeat=False,
)

# Save the combined animation as an MP4 file
combined_ani.save("combined_animation.mp4", writer="ffmpeg", fps=60)

# Show the animated plot
plt.show()
