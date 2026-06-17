from clearpath_config.common.utils.yaml import read_yaml
import rclpy
from rclpy.node import Node

from clearpath_control_selection_msgs.msg import ControlSelectionState

ROBOT_CONFIG_PATH = '/etc/clearpath/robot.yaml'
LOGGING_NAME = 'ControlSelectionState'


class ControlSelectionStateNode(Node):

    def __init__(self, namespace: str = ""):
        super().__init__('state_sub')
        self.state_sub = self.create_subscription(
            ControlSelectionState, f'/{namespace}/control_selection/control_state', self.state_cb, 10)
        self.state_sub  # prevent unused variable warning

    def state_cb(self, msg):
        self.last_state_msg = msg


def main(args=None):
    rclpy.init(args=args)
    robot_config = read_yaml(ROBOT_CONFIG_PATH)
    namespace = robot_config['system']['ros2']['namespace']

    state_sub = ControlSelectionStateNode(namespace)
    rclpy.spin(state_sub)


if __name__ == '__main__':
    main()
