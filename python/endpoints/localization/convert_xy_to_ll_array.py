from clearpath_config.common.utils.yaml import read_yaml
from clearpath_localization_msgs.srv import ConvertCartesianToLatLonArray
from geometry_msgs.msg import PoseStamped, Pose, Point
import rclpy
from rclpy.node import Node

ROBOT_CONFIG_PATH = '/etc/clearpath/robot.yaml'
LOGGING_NAME = 'ConvertCartesianToLatLonArray'

# TODO: Enter your Cartesian coordinates
POSE_MSG1 = PoseStamped(
    pose=Pose(
        position=Point(
            x=0.0,
            y=0.0,
            z=0.0,
        ),
    )
)
POSE_MSG2 = PoseStamped(
    pose=Pose(
        position=Point(
            x=0.0,
            y=0.0,
            z=0.0,
        ),
    )
)

class ConvertCartesianToLatLonArrayNode(Node):

    def __init__(self, namespace: str):
        super().__init__('convert_cartesian_to_lat_lon_array')
        self._convert_client = self.create_client(ConvertCartesianToLatLonArray, f'/{namespace}/localization/xy_to_lat_lon_array')

        while not self._convert_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info(f'[{LOGGING_NAME}] /{namespace}/localization/xy_to_lat_lon_array service not available, waiting again...')

    def convert(self, msg):
        req = ConvertCartesianToLatLonArray.Request()
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

    convert_client = ConvertCartesianToLatLonArrayNode(namespace)
    convert_client.convert([POSE_MSG1, POSE_MSG2])

    rclpy.spin(convert_client)


if __name__ == '__main__':
    main()
