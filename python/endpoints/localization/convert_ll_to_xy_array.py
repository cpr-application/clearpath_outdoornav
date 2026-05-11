from clearpath_config.common.utils.yaml import read_yaml
from clearpath_localization_msgs.srv import ConvertLatLonToCartesianArray
from sensor_msgs.msg import NavSatFix
import rclpy
from rclpy.node import Node

ROBOT_CONFIG_PATH = '/etc/clearpath/robot.yaml'
LOGGING_NAME = 'ConvertLatLonToCartesian'

# TODO: Enter your lat/lon coordinates
LAT_LON_MSG1 = NavSatFix(
            latitude=0.0,
            longitude=0.0,
)
LAT_LON_MSG2 = NavSatFix(
            latitude=0.0,
            longitude=0.0,
)

class ConvertLatLonToCartesianArrayNode(Node):

    def __init__(self, namespace: str):
        super().__init__('convert_lat_lon_to_cartesian_array')
        self._convert_client = self.create_client(ConvertLatLonToCartesianArray, f'/{namespace}/localization/lat_lon_to_xy_array')

        while not self._convert_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info(f'[{LOGGING_NAME}] /{namespace}/localization/lat_lon_to_xy_array service not available, waiting again...')

    def convert(self, msg_array):
        req = ConvertLatLonToCartesianArray.Request()
        req.msg = msg_array
        self.future = self._convert_client.call_async(req)
        self.future.add_done_callback(self.convert_srv_response_cb)

    def convert_srv_response_cb(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'[{LOGGING_NAME}] Converted lat/lon array to Cartesian successfully!')
            else:
                self.get_logger().error(f'[{LOGGING_NAME}] Failed to convert lat/lon array to Cartesian')
        except Exception as e:
            self.get_logger().error(f'[{LOGGING_NAME}] Service call failed: {e}')

        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    robot_config = read_yaml(ROBOT_CONFIG_PATH)
    namespace = robot_config['system']['ros2']['namespace']

    convert_client = ConvertLatLonToCartesianArrayNode(namespace)
    convert_client.convert([LAT_LON_MSG1, LAT_LON_MSG2])

    rclpy.spin(convert_client)


if __name__ == '__main__':
    main()
