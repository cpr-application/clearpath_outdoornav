from clearpath_config.common.utils.yaml import read_yaml
import rclpy
from rclpy.node import Node
from clearpath_dock_msgs.srv import GetDockPoses

ROBOT_CONFIG_PATH = '/etc/clearpath/robot.yaml'
LOGGING_NAME = 'GetDockPoses'


class GetDockPosesNode(Node):

    def __init__(self, namespace: str):
        super().__init__('get_dock_poses_client')
        self._get_dock_poses_client = self.create_client(GetDockPoses, f'/{namespace}/docking/dock_localizer/get_dock_poses')

        while not self._get_dock_poses_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info(f'[{LOGGING_NAME}] /{namespace}/docking/dock_localizer/get_dock_poses service not available, waiting again...')

    def get_poses(self):
        req = GetDockPoses.Request()
        self.future = self._get_dock_poses_client.call_async(req)
        self.future.add_done_callback(self.get_poses_srv_response_cb)

    def get_poses_srv_response_cb(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'[{LOGGING_NAME}] Docks retrieved successfully!')
            else:
                self.get_logger().error(f'[{LOGGING_NAME}] Failed to retrieve docks: {response.message}')
        except Exception as e:
            self.get_logger().error(f'[{LOGGING_NAME}] Service call failed: {e}')

        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    robot_config = read_yaml(ROBOT_CONFIG_PATH)
    namespace = robot_config['system']['ros2']['namespace']

    get_dock_poses_client = GetDockPosesNode(namespace)
    get_dock_poses_client.get_poses()

    rclpy.spin(get_dock_poses_client)


if __name__ == '__main__':
    main()
