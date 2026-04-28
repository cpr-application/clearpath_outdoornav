from clearpath_config.common.utils.yaml import read_yaml
import rclpy
from rclpy.node import Node
from clearpath_dock_msgs.srv import RemoveDock

ROBOT_CONFIG_PATH = '/etc/clearpath/robot.yaml'
LOGGING_NAME = 'DeleteDock'

# TODO: Enter the name of the dock to be removed
DOCK_NAME: str = ''


class DeleteDock(Node):

    def __init__(self, namespace: str):
        super().__init__('delete_dock_client')
        self._delete_dock_client = self.create_client(RemoveDock, f'/{namespace}/docking/dock_manager/delete_dock')

        while not self._delete_dock_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info(f'[{LOGGING_NAME}] /{namespace}/docking/dock_manager/delete_dock service not available, waiting again...')

    def delete_dock(self):
        req = RemoveDock.Request()
        req.name = DOCK_NAME
        self.future = self._delete_dock_client.call_async(req)
        self.future.add_done_callback(self.delete_dock_srv_response_cb)

    def delete_dock_srv_response_cb(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'[{LOGGING_NAME}] Dock deleted successfully!')
            else:
                self.get_logger().error(f'[{LOGGING_NAME}] Failed to delete dock: {response.message}')
        except Exception as e:
            self.get_logger().error(f'[{LOGGING_NAME}] Service call failed: {e}')

        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    robot_config = read_yaml(ROBOT_CONFIG_PATH)
    namespace = robot_config['system']['ros2']['namespace']

    delete_dock_client = DeleteDock(namespace)
    delete_dock_client.delete_dock()

    rclpy.spin(delete_dock_client)


if __name__ == '__main__':
    main()
