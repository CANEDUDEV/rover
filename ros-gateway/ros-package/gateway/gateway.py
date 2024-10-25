import argparse

import can
import rclpy  # pyright: ignore

import rover

from .battery import (
    BatteryCellVoltagePublisher,
    BatteryMonitor,
    BatteryOutput,
    BatteryOutputPublisher,
)
from .brake import Wheel, WheelSpeedPublisher
from .obstacle_detector import ObstacleDetector, ObstaclePublisher


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("logfile")
    args = parser.parse_args()

    log = can.LogReader(args.logfile)

    rclpy.init()

    bm_cs_cell_voltage_publisher = BatteryCellVoltagePublisher(
        BatteryMonitor.BATTERY_MONITOR_CONTROL_SYSTEM
    )
    bm_ad_cell_voltage_publisher = BatteryCellVoltagePublisher(
        BatteryMonitor.BATTERY_MONITOR_AD_SYSTEM
    )
    bm_cs_output_publisher = BatteryOutputPublisher(
        BatteryMonitor.BATTERY_MONITOR_CONTROL_SYSTEM, BatteryOutput.BATTERY_OUTPUT
    )
    bm_ad_output_publisher = BatteryOutputPublisher(
        BatteryMonitor.BATTERY_MONITOR_AD_SYSTEM, BatteryOutput.BATTERY_OUTPUT
    )

    bm_cs_reg_output_publisher = BatteryOutputPublisher(
        BatteryMonitor.BATTERY_MONITOR_CONTROL_SYSTEM,
        BatteryOutput.BATTERY_REGULATED_OUTPUT,
    )
    bm_ad_reg_output_publisher = BatteryOutputPublisher(
        BatteryMonitor.BATTERY_MONITOR_AD_SYSTEM, BatteryOutput.BATTERY_REGULATED_OUTPUT
    )

    ws_fl_publisher = WheelSpeedPublisher(Wheel.WHEEL_FRONT_LEFT)
    ws_fr_publisher = WheelSpeedPublisher(Wheel.WHEEL_FRONT_RIGHT)
    ws_rl_publisher = WheelSpeedPublisher(Wheel.WHEEL_REAR_LEFT)
    ws_rr_publisher = WheelSpeedPublisher(Wheel.WHEEL_REAR_RIGHT)

    od_front_publisher = ObstaclePublisher(ObstacleDetector.OBSTACLE_DETECTOR_FRONT)
    od_rear_publisher = ObstaclePublisher(ObstacleDetector.OBSTACLE_DETECTOR_REAR)

    for msg in log:
        if msg.arbitration_id == rover.Envelope.BATTERY_CELL_VOLTAGES:
            bm_cs_cell_voltage_publisher.publish(msg)
        elif msg.arbitration_id == rover.Envelope.BATTERY_REGULATED_OUTPUT:
            bm_cs_reg_output_publisher.publish(msg)
        elif msg.arbitration_id == rover.Envelope.BATTERY_OUTPUT:
            bm_cs_output_publisher.publish(msg)

        elif msg.arbitration_id == rover.Envelope.AD_BATTERY_CELL_VOLTAGES:
            bm_ad_cell_voltage_publisher.publish(msg)
        elif msg.arbitration_id == rover.Envelope.AD_BATTERY_REGULATED_OUTPUT:
            bm_ad_reg_output_publisher.publish(msg)
        elif msg.arbitration_id == rover.Envelope.AD_BATTERY_OUTPUT:
            bm_ad_output_publisher.publish(msg)

        elif msg.arbitration_id == rover.Envelope.WHEEL_FRONT_LEFT_SPEED:
            ws_fl_publisher.publish(msg)
        elif msg.arbitration_id == rover.Envelope.WHEEL_FRONT_RIGHT_SPEED:
            ws_fr_publisher.publish(msg)
        elif msg.arbitration_id == rover.Envelope.WHEEL_REAR_LEFT_SPEED:
            ws_rl_publisher.publish(msg)
        elif msg.arbitration_id == rover.Envelope.WHEEL_REAR_RIGHT_SPEED:
            ws_rr_publisher.publish(msg)

        elif msg.arbitration_id == rover.Envelope.OBSTACLE_DETECTOR_FRONT_DISTANCE:
            od_front_publisher.publish(msg)
        elif msg.arbitration_id == rover.Envelope.OBSTACLE_DETECTOR_REAR_DISTANCE:
            od_rear_publisher.publish(msg)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
