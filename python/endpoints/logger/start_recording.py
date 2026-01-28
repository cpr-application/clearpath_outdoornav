from clearpath_config.common.utils.yaml import read_yaml
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

ROBOT_CONFIG_PATH = '/etc/clearpath/robot.yaml'
LOGGING_NAME = 'StartLogger'


class Logger(Node):

    def __init__(self, namespace: str):
        super().__init__('start_logger_client')
        self._start_logger_client = self.create_client(Trigger, f'/{namespace}/log_manager/start_recording')

        while not self._start_logger_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info(f'[{LOGGING_NAME}] /{namespace}/log_manager/start_recording service not available, waiting again...')

    def start(self):
        req = Trigger.Request()
        self.future = self._start_logger_client.call_async(req)
        self.future.add_done_callback(self.start_logger_srv_response_cb)

    def start_logger_srv_response_cb(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'[{LOGGING_NAME}] Logger started!')
            else:
                self.get_logger().error(f'[{LOGGING_NAME}] Failed to start logger')
        except Exception as e:
            self.get_logger().error(f'[{LOGGING_NAME}] Service call failed: {e}')

        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    robot_config = read_yaml(ROBOT_CONFIG_PATH)
    namespace = robot_config['system']['ros2']['namespace']

    logger = Logger(namespace)
    logger.start()

    rclpy.spin(logger)


if __name__ == '__main__':
    main()
