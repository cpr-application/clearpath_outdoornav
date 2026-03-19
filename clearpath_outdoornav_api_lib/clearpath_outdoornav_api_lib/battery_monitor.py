#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

LOGGING_NAME = 'BatteryMonitor'


class BatteryMonitor:

    def __init__(self, node: Node, namespace: str = ""):
        self.node = node
        sub_topic = f'/{namespace}/platform/bms/state' if namespace else 'platform/bms/state'

        self.battery_sub = self.node.create_subscription(
            BatteryState, sub_topic, self.state_cb, rclpy.qos.qos_profile_sensor_data)
        self.state_cb  # prevent unused variable warning
        self._current_battery_percentage = 0.0

    def state_cb(self, msg: BatteryState):
        self._current_battery_percentage = msg.percentage

    def get_battery_percentage(self) -> float:
        return self._current_battery_percentage
