from clearpath_config.common.utils.yaml import read_yaml
import rclpy
from rclpy.node import Node
from clearpath_dock_msgs.srv import ImportData

ROBOT_CONFIG_PATH = '/etc/clearpath/robot.yaml'
LOGGING_NAME = 'ImportDocks'


class ImportDocks(Node):

    def __init__(self, namespace: str):
        super().__init__('import_docks_client')
        self._import_docks_client = self.create_client(ImportData, f'/{namespace}/docking/dock_manager/import')

        while not self._import_docks_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info(f'[{LOGGING_NAME}] /{namespace}/docking/dock_manager/import service not available, waiting again...')

    def import_docks(self):
        req = ImportData.Request()
        self.future = self._import_docks_client.call_async(req)
        self.future.add_done_callback(self.import_docks_srv_response_cb)

    def import_docks_srv_response_cb(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'[{LOGGING_NAME}] Docks imported successfully!')
            else:
                self.get_logger().error(f'[{LOGGING_NAME}] Failed to import docks: {response.message}')
        except Exception as e:
            self.get_logger().error(f'[{LOGGING_NAME}] Service call failed: {e}')

        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    robot_config = read_yaml(ROBOT_CONFIG_PATH)
    namespace = robot_config['system']['ros2']['namespace']

    import_docks_client = ImportDocks(namespace)
    import_docks_client.import_docks()

    rclpy.spin(import_docks_client)


if __name__ == '__main__':
    main()
