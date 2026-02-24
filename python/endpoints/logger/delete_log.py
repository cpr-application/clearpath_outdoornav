from clearpath_config.common.utils.yaml import read_yaml
import rclpy
from rclpy.node import Node
from clearpath_logger_msgs.srv import DeleteLog

ROBOT_CONFIG_PATH = '/etc/clearpath/robot.yaml'
LOGGING_NAME = 'DeleteLog'

# enter the log uuid to be deleted
LOG_ID: str = ''

class Logger(Node):

    def __init__(self, namespace: str):
        super().__init__('delete_log_client')
        self._start_logger_client = self.create_client(DeleteLog, f'/{namespace}/log_manager/delete_log')

        while not self._start_logger_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info(f'[{LOGGING_NAME}] /{namespace}/log_manager/delete_log service not available, waiting again...')

    def delete_log(self):
        req = DeleteLog.Request()
        req.uuid = LOG_ID
        req.delete_media = False
        req.purge_record = False
        self.future = self._start_logger_client.call_async(req)
        self.future.add_done_callback(self.delete_log_srv_response_cb)

    def delete_log_srv_response_cb(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'[{LOGGING_NAME}] Deleted log successfully!')
            else:
                self.get_logger().error(f'[{LOGGING_NAME}] Failed to delete log')
        except Exception as e:
            self.get_logger().error(f'[{LOGGING_NAME}] Service call failed: {e}')

        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    robot_config = read_yaml(ROBOT_CONFIG_PATH)
    namespace = robot_config['system']['ros2']['namespace']

    logger = Logger(namespace)
    logger.delete_log()

    rclpy.spin(logger)


if __name__ == '__main__':
    main()
