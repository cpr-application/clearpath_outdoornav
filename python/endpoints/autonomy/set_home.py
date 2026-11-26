# BETA: The home location API is in beta and will be available in the next release.

import math

from clearpath_config.common.utils.yaml import read_yaml
from clearpath_mission_manager_msgs.srv import SetHomeLocation
from clearpath_navigation_msgs.msg import HomeLocation
import rclpy
from rclpy.node import Node

ROBOT_CONFIG_PATH = '/etc/clearpath/robot.yaml'
LOGGING_NAME = 'SetHome'

# TODO: Set either a dock name (home is that dock) or a latitude/longitude (home is that location)
HOME_DOCK_NAME: str = ''
HOME_LATITUDE: float = 0.0
HOME_LONGITUDE: float = 0.0
# Final heading at a location home, in degrees counter-clockwise from east, or None for any heading
HOME_HEADING_DEG = None


class SetHomeClient(Node):

    def __init__(self, namespace: str):
        super().__init__('set_home_client')
        self._set_home_client = self.create_client(SetHomeLocation, f'/{namespace}/mission_manager/set_home')

        while not self._set_home_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info(f'[{LOGGING_NAME}] /{namespace}/mission_manager/set_home service not available, waiting again...')

    def set_home(self, home):
        req = SetHomeLocation.Request()
        req.home = home
        self.future = self._set_home_client.call_async(req)
        self.future.add_done_callback(self.set_home_srv_response_cb)

    def set_home_srv_response_cb(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'[{LOGGING_NAME}] {response.message}')
            else:
                self.get_logger().error(f'[{LOGGING_NAME}] Failed to set home: {response.message}')
        except Exception as e:
            self.get_logger().error(f'[{LOGGING_NAME}] Service call failed: {e}')

        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    robot_config = read_yaml(ROBOT_CONFIG_PATH)
    namespace = robot_config['system']['ros2']['namespace']

    home = HomeLocation()
    if HOME_DOCK_NAME:
        home.type = HomeLocation.TYPE_DOCK
        home.dock_name = HOME_DOCK_NAME
    else:
        home.type = HomeLocation.TYPE_LOCATION
        home.latitude = HOME_LATITUDE
        home.longitude = HOME_LONGITUDE
        home.enable_heading = HOME_HEADING_DEG is not None
        home.heading = math.radians(HOME_HEADING_DEG) if HOME_HEADING_DEG is not None else 0.0

    set_home_client = SetHomeClient(namespace)
    set_home_client.set_home(home)

    rclpy.spin(set_home_client)


if __name__ == '__main__':
    main()
