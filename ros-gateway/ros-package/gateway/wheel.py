import enum
import struct

import std_msgs.msg as msgtype  # pyright: ignore
from rclpy.node import Node  # pyright: ignore
from rclpy.qos import ReliabilityPolicy  # pyright: ignore

import rover

from .topic import rover_topic


@enum.unique
class Wheel(enum.IntEnum):
    WHEEL_FRONT_LEFT = 0
    WHEEL_FRONT_RIGHT = 1
    WHEEL_REAR_LEFT = 2
    WHEEL_REAR_RIGHT = 3


class Publisher(Node):
    def __init__(self, wheel):
        if wheel not in Wheel:
            raise ValueError()

        self.wheel = wheel
        self.name = wheel.name.lower()
        super().__init__(self.name)

        self.rpm_topic = f"{self.name}/rpm"
        self.speed_topic = f"{self.name}/speed_kph"

        self.rpm_publisher = self.create_publisher(
            msgtype.Float32, rover_topic(self.rpm_topic), ReliabilityPolicy.BEST_EFFORT
        )
        self.speed_publisher = self.create_publisher(
            msgtype.Float32,
            rover_topic(self.speed_topic),
            ReliabilityPolicy.BEST_EFFORT,
        )

    def __publish_rpm(self, msg):
        rpm_msg = msgtype.Float32()
        rpm_msg.data = struct.unpack("f", msg.data[0:4])[0]
        self.rpm_publisher.publish(rpm_msg)
        self.get_logger().info(f'Publishing {self.rpm_topic}: "{rpm_msg.data}"')

    def __publish_speed(self, msg):
        speed_msg = msgtype.Float32()
        speed_msg.data = struct.unpack("f", msg.data[4:8])[0]
        self.get_logger().info(f'Publishing {self.speed_topic}: "{speed_msg.data}"')
        self.speed_publisher.publish(speed_msg)

    def publish(self, msg):
        id = msg.arbitration_id

        if (
            (
                self.wheel == Wheel.WHEEL_FRONT_LEFT
                and id == rover.Envelope.WHEEL_FRONT_LEFT_SPEED
            )
            or (
                self.wheel == Wheel.WHEEL_FRONT_RIGHT
                and id == rover.Envelope.WHEEL_FRONT_RIGHT_SPEED
            )
            or (
                self.wheel == Wheel.WHEEL_REAR_LEFT
                and id == rover.Envelope.WHEEL_REAR_LEFT_SPEED
            )
            or (
                self.wheel == Wheel.WHEEL_REAR_RIGHT
                and id == rover.Envelope.WHEEL_REAR_RIGHT_SPEED
            )
        ):

            self.__publish_rpm(msg)
            self.__publish_speed(msg)
