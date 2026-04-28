from clearpath_config.common.utils.yaml import read_yaml
import rclpy
from rclpy.node import Node

from clearpath_navigation_msgs.msg import AutonomyStatus

ROBOT_CONFIG_PATH = '/etc/clearpath/robot.yaml'
LOGGING_NAME = 'AutonomyStatus'


class AutonomyStatusNode(Node):

    def __init__(self, namespace: str = ""):
        super().__init__('status_sub')
        self.status_sub = self.create_subscription(
            AutonomyStatus, f'/{namespace}/autonomy/status', self.status_cb, 10)
        self.status_sub  # prevent unused variable warning

    def status_cb(self, msg):
        self.last_status_msg = msg

def main(args=None):
    rclpy.init(args=args)
    robot_config = read_yaml(ROBOT_CONFIG_PATH)
    namespace = robot_config['system']['ros2']['namespace']

    status_sub = AutonomyStatusNode(namespace)
    rclpy.spin(status_sub)


if __name__ == '__main__':
    main()