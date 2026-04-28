from clearpath_config.common.utils.yaml import read_yaml
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped

ROBOT_CONFIG_PATH = '/etc/clearpath/robot.yaml'
LOGGING_NAME = 'UITeleopVel'


class UITeleopVelNode(Node):

    def __init__(self, namespace: str = ""):
        super().__init__('ui_teleop_vel_sub')
        self.teleop_vel_sub = self.create_subscription(
            TwistStamped, f'/{namespace}/ui_teleop/cmd_vel', self.teleop_vel_cb, 10)
        self.teleop_vel_sub  # prevent unused variable warning

    def teleop_vel_cb(self, msg):
        self.last_teleop_vel_msg = msg

def main(args=None):
    rclpy.init(args=args)
    robot_config = read_yaml(ROBOT_CONFIG_PATH)
    namespace = robot_config['system']['ros2']['namespace']

    teleop_vel_sub = UITeleopVelNode(namespace)
    rclpy.spin(teleop_vel_sub)


if __name__ == '__main__':
    main()