from clearpath_config.common.utils.yaml import read_yaml
import rclpy
from rclpy.node import Node

from clearpath_navigation_msgs.msg import AutonomyConfig

ROBOT_CONFIG_PATH = '/etc/clearpath/robot.yaml'
LOGGING_NAME = 'AutonomyConfiguration'


class AutonomyConfiguration(Node):

    def __init__(self, namespace: str = ""):
        super().__init__('config_sub')
        self.config_sub = self.create_subscription(
            AutonomyConfig, f'/{namespace}/outdoornav/autonomy/config', self.config_cb, 10)
        self.config_sub  # prevent unused variable warning

    def config_cb(self, msg):
        self.last_config_msg = msg


def main(args=None):
    rclpy.init(args=args)
    robot_config = read_yaml(ROBOT_CONFIG_PATH)
    namespace = robot_config['system']['ros2']['namespace']

    config_sub = AutonomyConfiguration(namespace)
    rclpy.spin(config_sub)


if __name__ == '__main__':
    main()