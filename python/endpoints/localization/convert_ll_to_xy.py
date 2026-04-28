from clearpath_config.common.utils.yaml import read_yaml
from clearpath_localization_msgs.srv import ConvertLatLonToCartesian
from sensor_msgs.msg import NavSatFix
import rclpy
from rclpy.node import Node

ROBOT_CONFIG_PATH = '/etc/clearpath/robot.yaml'
LOGGING_NAME = 'ConvertLatLonToCartesian'

# TODO: Enter your lat/lon coordinates
LAT_LON_MSG = NavSatFix(
            latitude=,
            longitude=,
)

class ConvertLatLonToCartesianNode(Node):

    def __init__(self, namespace: str):
        super().__init__('convert_lat_lon_to_cartesian')
        self._convert_client = self.create_client(ConvertLatLonToCartesian, f'/{namespace}/localization/lat_lon_to_xy')

        while not self._convert_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info(f'[{LOGGING_NAME}] /{namespace}/localization/lat_lon_to_xy service not available, waiting again...')

    def convert(self, msg):
        req = ConvertLatLonToCartesian.Request()
        req.msg = msg
        self.future = self._convert_client.call_async(req)
        self.future.add_done_callback(self.convert_srv_response_cb)

    def convert_srv_response_cb(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'[{LOGGING_NAME}] Converted lat/lon to Cartesian successfully!')
            else:
                self.get_logger().error(f'[{LOGGING_NAME}] Failed to convert lat/lon to Cartesian')
        except Exception as e:
            self.get_logger().error(f'[{LOGGING_NAME}] Service call failed: {e}')

        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    robot_config = read_yaml(ROBOT_CONFIG_PATH)
    namespace = robot_config['system']['ros2']['namespace']

    convert_client = ConvertLatLonToCartesianNode(namespace)
    convert_client.convert(LAT_LON_MSG)

    rclpy.spin(convert_client)


if __name__ == '__main__':
    main()
