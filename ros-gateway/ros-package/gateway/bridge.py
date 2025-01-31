import can
import rclpy  # pyright: ignore
import rclpy.logging  # pyright: ignore

import rover

from . import battery, obstacle_detector, radio, wheel
from .battery import BatteryMonitor
from .controller import RosController
from .obstacle_detector import ObstacleDetector
from .wheel import Wheel


class RosBridge(can.Listener):
    def __init__(self, verbosity="info"):
        rclpy.logging.set_logger_level(
            name="", level=rclpy.logging.get_logging_severity_from_string(verbosity)
        )
        self.logger = rclpy.logging.get_logger("rover")

        self.logger.info("initializing ROS2 bridge")

        self.can_bus = None
        self.can_notifier = None

        self.last_control_override_message_timestamp = 0

        rclpy.init()

        self.controller = RosController()

        self.publishers = [
            battery.Publisher(BatteryMonitor.BATTERY_MONITOR_CONTROL_SYSTEM),
            battery.Publisher(BatteryMonitor.BATTERY_MONITOR_AD_SYSTEM),
            wheel.Publisher(Wheel.WHEEL_FRONT_LEFT),
            wheel.Publisher(Wheel.WHEEL_FRONT_RIGHT),
            wheel.Publisher(Wheel.WHEEL_REAR_LEFT),
            wheel.Publisher(Wheel.WHEEL_REAR_RIGHT),
            obstacle_detector.Publisher(ObstacleDetector.OBSTACLE_DETECTOR_FRONT),
            obstacle_detector.Publisher(ObstacleDetector.OBSTACLE_DETECTOR_REAR),
            radio.Publisher(),
        ]

        self.logger.info("finished initialization")

    def __del__(self):
        self.logger.info("ROS2 bridge stopped, cleaning up")

        self.controller.destroy_node()

        for publisher in self.publishers:
            publisher.destroy_node()

        if self.can_notifier is not None:
            self.can_notifier.stop(timeout=10)

        if self.can_bus is not None:
            self.can_bus.shutdown()

        self.logger.info("finished cleanup")

    def on_message_received(self, msg):
        if (
            msg.arbitration_id == rover.Envelope.STEERING
            or msg.arbitration_id == rover.Envelope.THROTTLE
        ):
            self.controller.refresh_radio_timestamp(msg.timestamp)

        for publisher in self.publishers:
            publisher.publish(msg)

    def start(self):
        self.can_bus = can.ThreadSafeBus(
            interface="socketcan", channel="can0", bitrate=125_000
        )
        self.can_notifier = can.Notifier(self.can_bus, [self])

        self.controller.set_can_bus(self.can_bus)
        rclpy.spin(self.controller)

    def simulate(self, logfile):
        log = can.LogReader(logfile)
        for msg in log:
            self.on_message_received(msg)
