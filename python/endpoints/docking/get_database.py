from clearpath_config.common.utils.yaml import read_yaml
import rclpy
from rclpy.node import Node
from clearpath_dock_msgs.srv import GetDockDatabase

ROBOT_CONFIG_PATH = '/etc/clearpath/robot.yaml'
LOGGING_NAME = 'GetDockDatabase'


class GetDockDatabaseNode(Node):

    def __init__(self, namespace: str):
        super().__init__('get_dock_database_client')
        self._get_dock_database_client = self.create_client(GetDockDatabase, f'/{namespace}/docking/dock_manager/get_database')

        while not self._get_dock_database_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info(f'[{LOGGING_NAME}] /{namespace}/docking/dock_manager/get_database service not available, waiting again...')

    def get_database(self):
        req = GetDockDatabase.Request()
        self.future = self._get_dock_database_client.call_async(req)
        self.future.add_done_callback(self.get_database_srv_response_cb)

    def get_database_srv_response_cb(self, future):
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

    get_dock_database_client = GetDockDatabaseNode(namespace)
    get_dock_database_client.get_database()

    rclpy.spin(get_dock_database_client)


if __name__ == '__main__':
    main()
