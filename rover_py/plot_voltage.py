import argparse
import csv

import matplotlib.pyplot as plt


def plot_csv_data(csv_file_path):
    # Initialize empty lists to store timestamp and data values
    timestamps = []
    data_values = []

    # Open and read the CSV file
    with open(csv_file_path, "r") as csv_file:
        csv_reader = csv.DictReader(csv_file)
        for row in csv_reader:
            timestamps.append(float(row["timestamp"]))
            hex_data = row["data"]  # Assuming data is already in little-endian format
            reversed_hex_data = "".join(
                reversed([hex_data[i : i + 2] for i in range(0, len(hex_data), 2)])
            )
            data_values.append(
                int(reversed_hex_data, 16)
            )  # Convert hexadecimal string to integer

    # Plot the data
    plt.figure(figsize=(10, 6))
    plt.plot(timestamps, data_values, marker="o", linestyle="-", color="b")
    plt.xlabel("Timestamp")
    plt.ylabel("Voltage (mV)")
    plt.title(f"Voltage vs. Timestamp ({csv_file_path})")  # Add filename to the title
    plt.grid(True)

    # Show the plot
    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot data from a CSV file.")
    parser.add_argument("csv_file", help="Path to the CSV file containing data")
    args = parser.parse_args()

    plot_csv_data(args.csv_file)
