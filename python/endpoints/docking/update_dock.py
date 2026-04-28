from clearpath_config.common.utils.yaml import read_yaml
import rclpy
from rclpy.node import Node
from clearpath_dock_msgs.srv import UpdateDock

ROBOT_CONFIG_PATH = '/etc/clearpath/robot.yaml'
LOGGING_NAME = 'UpdateDock'


class UpdateDock(Node):

    def __init__(self, namespace: str):
        super().__init__('update_dock_client')
        self._update_dock_client = self.create_client(UpdateDock, f'/{namespace}/docking/dock_manager/update')

        while not self._update_dock_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info(f'[{LOGGING_NAME}] /{namespace}/docking/dock_manager/update service not available, waiting again...')

    def update_dock(self):
        req = UpdateDock.Request()
        self.future = self._update_dock_client.call_async(req)
        self.future.add_done_callback(self.update_dock_srv_response_cb)

    def update_dock_srv_response_cb(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'[{LOGGING_NAME}] Dock updated successfully!')
            else:
                self.get_logger().error(f'[{LOGGING_NAME}] Failed to update dock: {response.message}')
        except Exception as e:
            self.get_logger().error(f'[{LOGGING_NAME}] Service call failed: {e}')

        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    robot_config = read_yaml(ROBOT_CONFIG_PATH)
    namespace = robot_config['system']['ros2']['namespace']

    update_dock_client = UpdateDock(namespace)
    update_dock_client.update_dock()

    rclpy.spin(update_dock_client)


if __name__ == '__main__':
    main()
