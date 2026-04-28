from clearpath_config.common.utils.yaml import read_yaml
import rclpy
from rclpy.node import Node

from nav2_msgs.msg import SpeedLimit

ROBOT_CONFIG_PATH = '/etc/clearpath/robot.yaml'
LOGGING_NAME = 'SpeedLimit'


class SpeedLimitNode(Node):

    def __init__(self, namespace: str = ""):
        super().__init__('speed_limit_sub')
        self.speed_limit_sub = self.create_subscription(
            SpeedLimit, f'/{namespace}/navigation/speed_limit', self.speed_limit_cb, 10)
        self.speed_limit_sub  # prevent unused variable warning

    def speed_limit_cb(self, msg):
        self.last_speed_limit_msg = msg

def main(args=None):
    rclpy.init(args=args)
    robot_config = read_yaml(ROBOT_CONFIG_PATH)
    namespace = robot_config['system']['ros2']['namespace']

    speed_limit_sub = SpeedLimitNode(namespace)
    rclpy.spin(speed_limit_sub)


if __name__ == '__main__':
    main()