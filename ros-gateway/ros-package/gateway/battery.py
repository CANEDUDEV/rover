import enum
import struct

import std_msgs.msg as msgtype  # pyright: ignore
from rclpy.node import Node  # pyright: ignore
from rclpy.qos import ReliabilityPolicy  # pyright: ignore

import rover

from .topic import rover_topic


@enum.unique
class BatteryMonitor(enum.IntEnum):
    BATTERY_MONITOR_CONTROL_SYSTEM = 0
    BATTERY_MONITOR_AD_SYSTEM = 1


class Publisher(Node):
    def __init__(self, battery_monitor):
        if battery_monitor not in BatteryMonitor:
            raise ValueError()

        self.battery_monitor = battery_monitor
        self.name = battery_monitor.name.lower()
        super().__init__(self.name)

        self.get_logger().info(f"initializing {self.name}")

        self.cell_voltages_topic = f"{self.name}/cell_voltage_mV"
        self.cell_voltages_publisher = self.create_publisher(
            msgtype.UInt16MultiArray,
            rover_topic(self.cell_voltages_topic),
            ReliabilityPolicy.BEST_EFFORT,
        )
        self.cell_voltages = []

        self.voltage_topic = f"{self.name}/battery_output/voltage_mV"
        self.current_topic = f"{self.name}/battery_output/current_mA"
        self.voltage_publisher = self.create_publisher(
            msgtype.UInt32,
            rover_topic(self.voltage_topic),
            ReliabilityPolicy.BEST_EFFORT,
        )
        self.current_publisher = self.create_publisher(
            msgtype.UInt32,
            rover_topic(self.current_topic),
            ReliabilityPolicy.BEST_EFFORT,
        )

        self.reg_output_voltage_topic = f"{self.name}/regulated_output/voltage_mV"
        self.reg_output_current_topic = f"{self.name}/regulated_output/current_mA"
        self.reg_output_voltage_publisher = self.create_publisher(
            msgtype.UInt32,
            rover_topic(self.reg_output_voltage_topic),
            ReliabilityPolicy.BEST_EFFORT,
        )
        self.reg_output_current_publisher = self.create_publisher(
            msgtype.UInt32,
            rover_topic(self.reg_output_current_topic),
            ReliabilityPolicy.BEST_EFFORT,
        )

        self.get_logger().info("finished intialization")

    def __publish_cell_voltages(self, msg):
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
            self.get_logger().debug(
                f'Publishing {self.topic}: "{cell_voltage_msg.data}"'
            )

            self.cell_voltages.clear()

    def __publish_output(self, msg):
        voltage_msg = msgtype.UInt32()
        current_msg = msgtype.UInt32()

        voltage_msg.data = struct.unpack("I", msg.data[0:4])[0]
        current_msg.data = struct.unpack("I", msg.data[4:8])[0]

        self.voltage_publisher.publish(voltage_msg)
        self.current_publisher.publish(current_msg)
        self.get_logger().debug(
            f'Publishing {self.voltage_topic}: "{voltage_msg.data}"'
        )
        self.get_logger().debug(
            f'Publishing {self.current_topic}: "{current_msg.data}"'
        )

    def __publish_reg_output(self, msg):
        voltage_msg = msgtype.UInt32()
        current_msg = msgtype.UInt32()

        voltage_msg.data = struct.unpack("I", msg.data[0:4])[0]
        current_msg.data = struct.unpack("I", msg.data[4:8])[0]

        self.reg_output_voltage_publisher.publish(voltage_msg)
        self.reg_output_current_publisher.publish(current_msg)
        self.get_logger().debug(
            f'Publishing {self.reg_output_voltage_topic}: "{voltage_msg.data}"'
        )
        self.get_logger().debug(
            f'Publishing {self.reg_output_current_topic}: "{current_msg.data}"'
        )

    def publish(self, msg):
        id = msg.arbitration_id

        if self.battery_monitor == BatteryMonitor.BATTERY_MONITOR_CONTROL_SYSTEM:
            if id == rover.Envelope.BATTERY_CELL_VOLTAGES:
                self.__publish_cell_voltages(msg)
            if id == rover.Envelope.BATTERY_OUTPUT:
                self.__publish_output(msg)
            if id == rover.Envelope.BATTERY_REGULATED_OUTPUT:
                self.__publish_reg_output(msg)

        if self.battery_monitor == BatteryMonitor.BATTERY_MONITOR_AD_SYSTEM:
            if id == rover.Envelope.AD_BATTERY_CELL_VOLTAGES:
                self.__publish_cell_voltages(msg)
            if id == rover.Envelope.AD_BATTERY_OUTPUT:
                self.__publish_output(msg)
            if id == rover.Envelope.AD_BATTERY_REGULATED_OUTPUT:
                self.__publish_reg_output(msg)
