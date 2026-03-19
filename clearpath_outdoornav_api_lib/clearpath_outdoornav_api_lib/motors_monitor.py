#!/usr/bin/python3

from clearpath_motor_msgs.msg import LynxMotorProtection, LynxSystemProtection
import rclpy
from rclpy.node import Node

LOGGING_NAME = 'MotorsMonitor'


class MotorsMonitor:

    def __init__(self, node: Node, namespace: str = ""):
        self.node = node
        sub_topic = f'/{namespace}/platform/motors/system_protection' if namespace else 'platform/motors/system_protection'

        self.motor_protection = self.node.create_subscription(
            LynxSystemProtection, sub_topic, self.motor_states_cb, rclpy.qos.qos_profile_sensor_data)
        self.motor_states_cb  # prevent unused variable warning
        self._motor_states = LynxMotorProtection.NORMAL

    def motor_states_cb(self, msg: LynxSystemProtection):
        self._motor_states = msg.system_state

    def get_states(self) -> int:
        return self._motor_states
