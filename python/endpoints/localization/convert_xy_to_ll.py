from clearpath_config.common.utils.yaml import read_yaml
from clearpath_localization_msgs.srv import ConvertCartesianToLatLon
from geometry_msgs.msg import PoseStamped, Pose, Point
import rclpy
from rclpy.node import Node

ROBOT_CONFIG_PATH = '/etc/clearpath/robot.yaml'
LOGGING_NAME = 'ConvertCartesianToLatLon'

# TODO: Enter your Cartesian coordinates
CARTESIAN_MSG = PoseStamped(
    pose=Pose(
        position=Point(
            x=,
            y=,
            z=,
        ),
    )
)


class ConvertCartesianToLatLonNode(Node):

    def __init__(self, namespace: str):
        super().__init__('convert_cartesian_to_lat_lon')
        self._convert_client = self.create_client(ConvertCartesianToLatLon, f'/{namespace}/localization/xy_to_lat_lon')

        while not self._convert_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info(f'[{LOGGING_NAME}] /{namespace}/localization/xy_to_lat_lon service not available, waiting again...')

    def convert(self, msg):
        req = ConvertCartesianToLatLon.Request()
        req.msg = msg
        self.future = self._convert_client.call_async(req)
        self.future.add_done_callback(self.convert_srv_response_cb)

    def convert_srv_response_cb(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'[{LOGGING_NAME}] Converted Cartesian to lat/lon successfully!')
            else:
                self.get_logger().error(f'[{LOGGING_NAME}] Failed to convert Cartesian to lat/lon')
        except Exception as e:
            self.get_logger().error(f'[{LOGGING_NAME}] Service call failed: {e}')

        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    robot_config = read_yaml(ROBOT_CONFIG_PATH)
    namespace = robot_config['system']['ros2']['namespace']

    convert_client = ConvertCartesianToLatLonNode(namespace)
    convert_client.convert(CARTESIAN_MSG)

    rclpy.spin(convert_client)


if __name__ == '__main__':
    main()
