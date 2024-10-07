import csv

from canlib import canlib

from rover import rover

# Initialize CANlib for both signals
with canlib.openChannel(
    channel=0, flags=canlib.Open.REQUIRE_INIT_ACCESS, bitrate=canlib.canBITRATE_125K
) as ch:
    ch.setBusOutputControl(canlib.Driver.NORMAL)
    ch.busOn()

    # Save collected data to CSV files
    with open("cells.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["Cell 1", "Cell 2", "Cell 3", "Cell 4"])

        try:
            cell1 = 0
            cell2 = 0
            cell3 = 0
            cell4 = 0
            while True:
                frame = ch.read(timeout=-1)
                if frame is None:
                    continue

                if frame.id == rover.Envelope.AD_BATTERY_CELL_VOLTAGES:
                    if frame.data[0] == 0:
                        cell1 = int.from_bytes(
                            bytes(frame.data[1:3]), byteorder="little"
                        )
                        cell2 = int.from_bytes(
                            bytes(frame.data[3:5]), byteorder="little"
                        )
                        cell3 = int.from_bytes(
                            bytes(frame.data[5:]), byteorder="little"
                        )
                    if frame.data[0] == 1:
                        cell4 = int.from_bytes(
                            bytes(frame.data[1:3]), byteorder="little"
                        )

                if cell1 > 0 and cell2 > 0 and cell3 > 0 and cell4 > 0:
                    writer.writerow([cell1, cell2, cell3, cell4])
                    cell1 = 0
                    cell2 = 0
                    cell3 = 0
                    cell4 = 0

        except KeyboardInterrupt:
            pass

        except Exception as e:
            print("Error:", e)

print("done")
