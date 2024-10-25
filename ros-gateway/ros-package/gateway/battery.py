import enum
import struct

import std_msgs.msg as msgtype  # pyright: ignore
from rclpy.node import Node  # pyright: ignore
from rclpy.qos import ReliabilityPolicy  # pyright: ignore

from .topic import rover_topic


@enum.unique
class BatteryMonitor(enum.IntEnum):
    BATTERY_MONITOR_CONTROL_SYSTEM = 0
    BATTERY_MONITOR_AD_SYSTEM = 1


class BatteryCellVoltagePublisher(Node):
    def __init__(self, battery_monitor):
        self.name = battery_monitor.name.lower()
        super().__init__(self.name)

        self.topic = f"{self.name}/cell_voltage_mV"
        self.publisher = self.create_publisher(
            msgtype.UInt16MultiArray,
            rover_topic(self.topic),
            ReliabilityPolicy.BEST_EFFORT,
        )
        self.cell_voltages = []

    def publish(self, msg):
        # If first message received is for the last 3 cells, skip it
        # If something went wrong with the reception order, we also skip it
        if (msg.data[0] == 1 and len(self.cell_voltages) <= 3) or (
            msg.data[0] == 0 and len(self.cell_voltages) > 0
        ):
            self.cell_voltages.clear()
            return

        self.cell_voltages.append(struct.unpack("H", msg.data[1:3])[0])
        self.cell_voltages.append(struct.unpack("H", msg.data[3:5])[0])
        self.cell_voltages.append(struct.unpack("H", msg.data[5:7])[0])

        if len(self.cell_voltages) >= 6:
            cell_voltage_msg = msgtype.UInt16MultiArray()
            cell_voltage_msg.data = self.cell_voltages
            self.publisher.publish(cell_voltage_msg)
            self.get_logger().info(
                f'Publishing {self.topic}: "{cell_voltage_msg.data}"'
            )

            self.cell_voltages.clear()


@enum.unique
class BatteryOutput(enum.IntEnum):
    BATTERY_OUTPUT = 0
    BATTERY_REGULATED_OUTPUT = 1


class BatteryOutputPublisher(Node):
    def __init__(self, battery_monitor, output_type):
        self.name = battery_monitor.name.lower()
        super().__init__(self.name)

        self.output_type = output_type.name.lower()

        self.voltage_topic = f"{self.name}/{self.output_type}/voltage_mV"
        self.current_topic = f"{self.name}/{self.output_type}/current_mA"
        self.voltage_publisher = self.create_publisher(
            msgtype.UInt32,
            rover_topic(self.voltage_topic),
            ReliabilityPolicy.BEST_EFFORT,
        )
        self.current_publisher = self.create_publisher(
            msgtype.UInt32,
            rover_topic(self.voltage_topic),
            ReliabilityPolicy.BEST_EFFORT,
        )

    def __publish_voltage(self, msg):
        voltage_msg = msgtype.UInt32()
        voltage_msg.data = struct.unpack("I", msg.data[0:4])[0]
        self.voltage_publisher.publish(voltage_msg)
        self.get_logger().info(f'Publishing {self.voltage_topic}: "{voltage_msg.data}"')

    def __publish_current(self, msg):
        current_msg = msgtype.UInt32()
        current_msg.data = struct.unpack("I", msg.data[4:8])[0]
        self.voltage_publisher.publish(current_msg)
        self.get_logger().info(f'Publishing {self.current_topic}: "{current_msg.data}"')

    def publish(self, msg):
        self.__publish_voltage(msg)
        self.__publish_current(msg)
