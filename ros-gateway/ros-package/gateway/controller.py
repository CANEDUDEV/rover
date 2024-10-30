import struct

import can
import std_msgs.msg as msgtype  # pyright: ignore
from rclpy.node import Node  # pyright: ignore
from rclpy.qos import ReliabilityPolicy  # pyright: ignore

import rover

from .topic import rover_topic


class RosController(Node):
    def __init__(self):
        super().__init__("ad_controller")
        self.throttle_sub = self.create_subscription(
            msgtype.Float32,
            rover_topic("throttle"),
            self.__throttle_callback,
            ReliabilityPolicy.BEST_EFFORT,
        )

        self.steering_sub = self.create_subscription(
            msgtype.Float32,
            rover_topic("steering"),
            self.__steering_callback,
            ReliabilityPolicy.BEST_EFFORT,
        )

        self.can_bus = None

        self.override = True
        self.throttle = 1500
        self.steering = 0
        self.control_freq_hz = 100
        self.control_timer = self.create_timer(
            1 / self.control_freq_hz, self.send_control_command
        )

    def set_can_bus(self, can_bus):
        self.can_bus = can_bus

    # 1000-2000, 1500 is neutral
    def throttle_message(self):
        throttle_pulse = round(1500 + 5 * self.throttle)
        return can.Message(
            arbitration_id=rover.Envelope.THROTTLE,
            data=[0] + list(struct.pack("H", throttle_pulse)),
            is_extended_id=False,
        )

    def steering_message(self):
        return can.Message(
            arbitration_id=rover.Envelope.THROTTLE,
            data=struct.pack("f", self.steering),
            is_extended_id=False,
        )

    def set_override(self):
        self.override = True

    def clear_override(self):
        self.override = False

    def send_control_command(self):
        if self.override:
            self.get_logger().info(f"radio control override active, will not interfere")
            return

        self.get_logger().info(
            f"sending CAN control command (throttle: {self.throttle}, steering: {self.steering})"
        )
        if self.can_bus is not None:
            self.can_bus.send(self.throttle_message())
            self.can_bus.send(self.steering_message())

    def __throttle_callback(self, msg):
        # Negative values are reverse, positive are forward
        throttle = msg.data
        self.get_logger().info(f"Received throttle: {throttle}")

        if throttle > 100:
            self.get_logger().warn(
                f"Received invalid throttle value: {msg.data}, limiting to 100"
            )
            throttle = 100

        if throttle < -100:
            self.get_logger().warn(
                f"Received invalid throttle value: {msg.data}, limiting to -100"
            )
            throttle = 0

        self.throttle = throttle

    def __steering_callback(self, msg):
        steering = msg.data
        self.get_logger().info(f"Received steering: {steering}")

        if steering > 45:
            self.get_logger().warn(
                f"Received invalid steering value: {msg.data}, limiting to 45"
            )
            steering = 45

        if steering < -45:  # ignore reverse for now
            self.get_logger().warn(
                f"Received invalid steering value: {msg.data}, limiting to -45"
            )
            steering = -45

        self.steering = steering
