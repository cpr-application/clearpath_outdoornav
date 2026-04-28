from clearpath_config.common.utils.yaml import read_yaml
import rclpy
from rclpy.node import Node

from clearpath_navigation_msgs.msg import MissionPreview

ROBOT_CONFIG_PATH = '/etc/clearpath/robot.yaml'
LOGGING_NAME = 'MissionPreview'


class MissionPreviewNode(Node):

    def __init__(self, namespace: str = ""):
        super().__init__('mission_preview_sub')
        self.preview_sub = self.create_subscription(
            MissionPreview, f'/{namespace}/autonomy_previewer/mission/preview', self.preview_cb, 10)
        self.preview_sub  # prevent unused variable warning

    def preview_cb(self, msg):
        self.last_preview_msg = msg

def main(args=None):
    rclpy.init(args=args)
    robot_config = read_yaml(ROBOT_CONFIG_PATH)
    namespace = robot_config['system']['ros2']['namespace']

    preview_sub = MissionPreviewNode(namespace)
    rclpy.spin(preview_sub)


if __name__ == '__main__':
    main()
