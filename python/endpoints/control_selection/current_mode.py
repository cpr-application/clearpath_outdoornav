from clearpath_config.common.utils.yaml import read_yaml
import rclpy
from rclpy.node import Node

from clearpath_control_selection_msgs.msg import ControlMode

ROBOT_CONFIG_PATH = '/etc/clearpath/robot.yaml'
LOGGING_NAME = 'CurrentControlMode'


class CurrentMode(Node):

    def __init__(self, namespace: str = ""):
        super().__init__('mode_sub')
        self.mode_sub = self.create_subscription(
            ControlMode, f'/{namespace}/control_selection/current_mode', self.mode_cb, 10)
        self.mode_sub  # prevent unused variable warning

    def mode_cb(self, msg):
        self.last_mode_msg = msg


def main(args=None):
    rclpy.init(args=args)
    robot_config = read_yaml(ROBOT_CONFIG_PATH)
    namespace = robot_config['system']['ros2']['namespace']

    mode_sub = CurrentMode(namespace)
    rclpy.spin(mode_sub)


if __name__ == '__main__':
    main()
