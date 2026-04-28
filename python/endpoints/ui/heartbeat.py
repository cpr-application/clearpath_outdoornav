from clearpath_config.common.utils.yaml import read_yaml
import rclpy
from rclpy.node import Node

from std_msgs.msg import Header

ROBOT_CONFIG_PATH = '/etc/clearpath/robot.yaml'
LOGGING_NAME = 'UIHeartbeat'


class UIHeartbeatNode(Node):

    def __init__(self, namespace: str = ""):
        super().__init__('ui_heartbeat_sub')
        self.heartbeat_sub = self.create_subscription(
            Header, f'/{namespace}/ui/heartbeat', self.heartbeat_cb, 10)
        self.heartbeat_sub  # prevent unused variable warning

    def heartbeat_cb(self, msg):
        self.last_heartbeat_msg = msg

def main(args=None):
    rclpy.init(args=args)
    robot_config = read_yaml(ROBOT_CONFIG_PATH)
    namespace = robot_config['system']['ros2']['namespace']

    heartbeat_sub = UIHeartbeatNode(namespace)
    rclpy.spin(heartbeat_sub)


if __name__ == '__main__':
    main()