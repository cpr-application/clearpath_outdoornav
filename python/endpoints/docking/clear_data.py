from clearpath_config.common.utils.yaml import read_yaml
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

ROBOT_CONFIG_PATH = '/etc/clearpath/robot.yaml'
LOGGING_NAME = 'ClearDockData'


class ClearDockData(Node):

    def __init__(self, namespace: str):
        super().__init__('clear_dock_data_client')
        self._clear_dock_data_client = self.create_client(Trigger, f'/{namespace}/docking/dock_manager/clear_data')

        while not self._clear_dock_data_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info(f'[{LOGGING_NAME}] /{namespace}/docking/dock_manager/clear_data service not available, waiting again...')

    def clear_data(self):
        req = Trigger.Request()
        self.future = self._clear_dock_data_client.call_async(req)
        self.future.add_done_callback(self.clear_data_srv_response_cb)

    def clear_data_srv_response_cb(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'[{LOGGING_NAME}] Dock data cleared successfully!')
            else:
                self.get_logger().error(f'[{LOGGING_NAME}] Failed to clear dock data: {response.message}')
        except Exception as e:
            self.get_logger().error(f'[{LOGGING_NAME}] Service call failed: {e}')

        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    robot_config = read_yaml(ROBOT_CONFIG_PATH)
    namespace = robot_config['system']['ros2']['namespace']

    clear_dock_data_client = ClearDockData(namespace)
    clear_dock_data_client.clear_data()

    rclpy.spin(clear_dock_data_client)


if __name__ == '__main__':
    main()
