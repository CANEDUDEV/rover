import struct
import time

import can
import std_msgs.msg as msgtype  # pyright: ignore
from rclpy.node import Node  # pyright: ignore
from rclpy.qos import ReliabilityPolicy  # pyright: ignore

import rover

from .topic import rover_topic


class RosController(Node):
    def __init__(self):
        super().__init__("ad_controller")

        self.get_logger().info("initializing ad_controller")

        self.throttle_sub = self.create_subscription(
            msgtype.Float32,
            rover_topic("throttle"),
            self.throttle_callback,
            ReliabilityPolicy.BEST_EFFORT,
        )

        self.steering_sub = self.create_subscription(
            msgtype.Float32,
            rover_topic("steering"),
            self.steering_callback,
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
        self.get_logger().info("finished initialization")

    def stop_timer(self):
        self.control_timer.destroy()

    def start_timer(self):
        self.control_timer = self.create_timer(
            1 / self.control_freq_hz, self.send_control_command
        )

    def set_can_bus(self, can_bus):
        self.can_bus = can_bus

    # 1000-2000µs pulse width, 1500 is neutral
    def throttle_message(self):
        throttle_pulse = round(1500 + 5 * self.throttle)
        return can.Message(
            arbitration_id=rover.Envelope.THROTTLE,
            data=[0] + list(struct.pack("i", throttle_pulse)),
            is_extended_id=False,
        )

    def steering_message(self):
        return can.Message(
            arbitration_id=rover.Envelope.STEERING,
            data=[1] + list(struct.pack("f", self.steering)),
            is_extended_id=False,
        )

    def set_override(self):
        self.override = True

    def clear_override(self):
        self.override = False

    def send_control_command(self):
        if self.override:
            self.get_logger().debug(
                f"radio control override active, will not interfere"
            )
            self.stop_timer()
            time.sleep(1)
            self.start_timer()
            return

        if self.can_bus is not None:
            try:
                self.can_bus.send(self.throttle_message())
                self.can_bus.send(self.steering_message())
                self.get_logger().debug(
                    f"sent CAN control command (throttle: {self.throttle}, steering: {self.steering})"
                )
            except can.exceptions.CanOperationError:
                self.get_logger().error(
                    f"""CAN error: steering command not sent. Retrying in 1 s.
    Potential causes: Buffer overflow, error frame, less than 2 nodes on bus, or invalid bitrate setting"""
                )
                self.can_bus.flush_tx_buffer()

                self.stop_timer()
                time.sleep(1)
                self.start_timer()

    def manual_throttle_command(self, throttle):
        self.throttle = throttle

        t = time.time()
        while time.time() - t < 1:
            time.sleep(0.01)
            self.send_control_command()

    def throttle_callback(self, msg):
        # Negative values are reverse, positive are forward
        throttle = msg.data
        self.get_logger().debug(f"Received throttle: {throttle}")

        switch_reverse = False
        switch_forward = False

        if self.throttle > 0 and throttle < 0:
            switch_reverse = True
        if self.throttle < 0 and throttle > 0:
            switch_forward = True

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

        # Reversing directions needs special treatment
        if switch_reverse or switch_forward:
            self.get_logger().debug(f"Reversing")

            self.stop_timer()

            # Braking required when switching from forward to reverse
            if switch_reverse:
                self.manual_throttle_command(-50)

            # Add extra neutral command so timing is equal when switching from reverse to forward
            if switch_forward:
                self.manual_throttle_command(0)

            self.manual_throttle_command(0)

            self.start_timer()

        self.throttle = throttle

    def steering_callback(self, msg):
        steering = msg.data
        self.get_logger().debug(f"Received steering: {steering}")

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
