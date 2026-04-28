from clearpath_config.common.utils.yaml import read_yaml
import rclpy
from rclpy.node import Node
from clearpath_dock_msgs.srv import ExportData

ROBOT_CONFIG_PATH = '/etc/clearpath/robot.yaml'
LOGGING_NAME = 'ExportDocks'


class DeleteDock(Node):

    def __init__(self, namespace: str):
        super().__init__('export_docks_client')
        self._export_docks_client = self.create_client(ExportData, f'/{namespace}/docking/dock_manager/export_docks')

        while not self._export_docks_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info(f'[{LOGGING_NAME}] /{namespace}/docking/dock_manager/export_docks service not available, waiting again...')

    def export_docks(self):
        req = ExportData.Request()
        self.future = self._export_docks_client.call_async(req)
        self.future.add_done_callback(self.export_docks_srv_response_cb)

    def export_docks_srv_response_cb(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'[{LOGGING_NAME}] Docks exported successfully!')
            else:
                self.get_logger().error(f'[{LOGGING_NAME}] Failed to export docks: {response.message}')
        except Exception as e:
            self.get_logger().error(f'[{LOGGING_NAME}] Service call failed: {e}')

        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    robot_config = read_yaml(ROBOT_CONFIG_PATH)
    namespace = robot_config['system']['ros2']['namespace']

    export_docks_client = DeleteDock(namespace)
    export_docks_client.export_docks()

    rclpy.spin(export_docks_client)


if __name__ == '__main__':
    main()
