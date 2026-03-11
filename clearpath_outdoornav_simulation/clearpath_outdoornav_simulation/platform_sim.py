#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from clearpath_motor_msgs.msg import LynxSystemProtection
from rclpy.qos import QoSProfile, ReliabilityPolicy
from clearpath_config.common.utils.yaml import read_yaml

LOGGING_NAME = 'PlatformSim'
ROBOT_CONFIG_PATH = '/etc/clearpath/robot.yaml'

class PlatformSim(Node):
    def __init__(self, namespace):
        super().__init__('platform_sim', namespace=namespace)

        # Create QoS profile for better reliability
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE)

        # Initialize subscribers and publishers
        self.battery_state_pub = self.create_publisher(
            BatteryState,
            'platform/bms/state',
            qos_profile)

        self.motor_protection_pub = self.create_publisher(
            LynxSystemProtection,
            'platform/motors/system_protection',
            qos_profile)

        self.create_timer(0.4, self.battery_state_timer_callback)
        self.create_timer(1.0, self.motor_protection_timer_callback)

        self.get_logger().info('Platform sim node Started')

    def battery_state_timer_callback(self):
        # Create message
        state_msg = BatteryState()
        state_msg.percentage = 1.0

        # Publish
        self.battery_state_pub.publish(state_msg)

    def motor_protection_timer_callback(self):
        # Create message
        protection_msg = LynxSystemProtection()
        protection_msg.system_state = 0

        # Publish
        self.motor_protection_pub.publish(protection_msg)


def main(args=None):
    rclpy.init(args=args)
    robot_config = read_yaml(ROBOT_CONFIG_PATH)
    namespace = robot_config['system']['ros2']['namespace']
    node = PlatformSim(namespace)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
