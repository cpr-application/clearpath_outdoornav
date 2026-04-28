from clearpath_config.common.utils.yaml import read_yaml
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path

ROBOT_CONFIG_PATH = '/etc/clearpath/robot.yaml'
LOGGING_NAME = 'AutonomyInitialPath'


class AutonomyInitialPath(Node):

    def __init__(self, namespace: str = ""):
        super().__init__('path_sub')
        self.path_sub = self.create_subscription(
            Path, f'/{namespace}/autonomy/initial_path', self.path_cb, 10)
        self.path_sub  # prevent unused variable warning

    def path_cb(self, msg):
        self.last_path_msg = msg

def main(args=None):
    rclpy.init(args=args)
    robot_config = read_yaml(ROBOT_CONFIG_PATH)
    namespace = robot_config['system']['ros2']['namespace']

    path_sub = AutonomyInitialPath(namespace)
    rclpy.spin(path_sub)


if __name__ == '__main__':
    main()
