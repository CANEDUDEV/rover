import enum
import struct

import std_msgs.msg as msgtype  # pyright: ignore
from rclpy.node import Node  # pyright: ignore
from rclpy.qos import ReliabilityPolicy  # pyright: ignore

from .topic import rover_topic


@enum.unique
class ObstacleDetector(enum.IntEnum):
    OBSTACLE_DETECTOR_FRONT = 0
    OBSTACLE_DETECTOR_REAR = 1


class ObstaclePublisher(Node):
    def __init__(self, obstacle_detector):
        self.name = obstacle_detector.name.lower()
        super().__init__(self.name)

        self.topic = f"{self.name}/distance_mm"
        self.publisher = self.create_publisher(
            msgtype.UInt16MultiArray,
            rover_topic(self.topic),
            ReliabilityPolicy.BEST_EFFORT,
        )

    def publish(self, msg):
        distance_msg = msgtype.UInt16MultiArray()
        distance_msg.data = [
            struct.unpack("H", msg.data[0:2])[0],
            struct.unpack("H", msg.data[2:4])[0],
            struct.unpack("H", msg.data[4:6])[0],
            struct.unpack("H", msg.data[6:8])[0],
        ]
        self.publisher.publish(distance_msg)
        self.get_logger().info(f'Publishing {self.topic}: "{distance_msg.data}"')
