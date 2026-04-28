from clearpath_config.common.utils.yaml import read_yaml
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix

ROBOT_CONFIG_PATH = '/etc/clearpath/robot.yaml'
LOGGING_NAME = 'GetDatum'


class GetDatum(Node):

    def __init__(self, namespace: str = ""):
        super().__init__('datum_sub')
        self.datum_sub = self.create_subscription(
            NavSatFix, f'/{namespace}/localization/datum', self.datum_cb, 10)
        self.datum_sub  # prevent unused variable warning

    def datum_cb(self, msg):
        self.last_datum_msg = msg

def main(args=None):
    rclpy.init(args=args)
    robot_config = read_yaml(ROBOT_CONFIG_PATH)
    namespace = robot_config['system']['ros2']['namespace']

    datum_sub = GetDatum(namespace)
    rclpy.spin(datum_sub)


if __name__ == '__main__':
    main()
