from clearpath_config.common.utils.yaml import read_yaml
from clearpath_dock_msgs.srv import AddDockCurrentPose
import rclpy
from rclpy.node import Node

ROBOT_CONFIG_PATH = '/etc/clearpath/robot.yaml'
LOGGING_NAME = 'AddDockCurrentPose'

# TODO: Enter your dock info
DOCK_NAME: str = ''
DOCK_TEMPLATE: str = 'a300_side_dock'


class AddDockCurrentPoseClient(Node):

    def __init__(self, namespace: str):
        super().__init__('add_dock_current_pose_client')
        self._add_dock_client = self.create_client(AddDockCurrentPose, f'/{namespace}/docking/dock_localizer/add_dock_current_pose')

        while not self._add_dock_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info(f'[{LOGGING_NAME}] /{namespace}/docking/dock_localizer/add_dock_current_pose service not available, waiting again...')

    def add_dock(self, dock_info):
        req = AddDockCurrentPose.Request()
        req.dock = dock_info
        self.future = self._add_dock_client.call_async(req)
        self.future.add_done_callback(self.add_dock_srv_response_cb)

    def add_dock_srv_response_cb(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'[{LOGGING_NAME}] Added dock successfully!')
            else:
                self.get_logger().error(f'[{LOGGING_NAME}] Failed to add dock')
        except Exception as e:
            self.get_logger().error(f'[{LOGGING_NAME}] Service call failed: {e}')

        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    robot_config = read_yaml(ROBOT_CONFIG_PATH)
    namespace = robot_config['system']['ros2']['namespace']

    add_dock_client = AddDockCurrentPoseClient(namespace)
    add_dock_client.add_dock(DOCK_INFO)

    rclpy.spin(add_dock_client)


if __name__ == '__main__':
    main()
