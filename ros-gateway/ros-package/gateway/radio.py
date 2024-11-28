import struct

import std_msgs.msg as msgtype  # pyright: ignore
from rclpy.node import Node  # pyright: ignore
from rclpy.qos import ReliabilityPolicy  # pyright: ignore

import rover

from .topic import rover_topic


class Publisher(Node):
    def __init__(self):
        self.name = "radio"
        super().__init__(self.name)

        self.get_logger().info(f"initializing {self.name}")

        self.throttle_topic = f"{self.name}/throttle"
        self.steering_topic = f"{self.name}/steering"
        self.throttle_publisher = self.create_publisher(
            msgtype.Float32,
            rover_topic(self.throttle_topic),
            ReliabilityPolicy.BEST_EFFORT,
        )
        self.steering_publisher = self.create_publisher(
            msgtype.Float32,
            rover_topic(self.steering_topic),
            ReliabilityPolicy.BEST_EFFORT,
        )

        self.get_logger().info(f"finished initialization")

    def __publish_throttle(self, msg):
        throttle_msg = msgtype.Float32()

        throttle_pulse = struct.unpack("H", msg.data[1:3])[0]
        throttle = round((throttle_pulse - 1500) / 5)
        if throttle > 100:
            throttle = 100
        if throttle < -100:
            throttle = -100

        throttle_msg.data = float(throttle)
        self.get_logger().debug(
            f'Publishing {self.throttle_topic}: "{throttle_msg.data}"'
        )
        self.throttle_publisher.publish(throttle_msg)

    def __publish_steering(self, msg):
        steering_msg = msgtype.Float32()
        steering_msg.data = struct.unpack("f", msg.data[1:5])[0]
        self.get_logger().debug(
            f'Publishing {self.steering_topic}: "{steering_msg.data}"'
        )
        self.steering_publisher.publish(steering_msg)

    def publish(self, msg):
        id = msg.arbitration_id
        if id == rover.Envelope.THROTTLE:
            self.__publish_throttle(msg)
        elif id == rover.Envelope.STEERING:
            self.__publish_steering(msg)
