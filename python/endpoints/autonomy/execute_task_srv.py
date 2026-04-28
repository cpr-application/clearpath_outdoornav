from clearpath_config.common.utils.yaml import read_yaml
from clearpath_task_msgs.srv import ExecuteTask
import rclpy
from rclpy.node import Node

ROBOT_CONFIG_PATH = '/etc/clearpath/robot.yaml'
LOGGING_NAME = 'ExecuteTaskSrv'

# TODO: Enter your task uuid
TASK_ID: str = ''


class ExecuteTaskSrv(Node):

    def __init__(self, namespace: str):
        super().__init__('execute_task_srv')
        self._execute_task_srv_client = self.create_client(ExecuteTask, f'/{namespace}/autonomy/task/execute_srv')

        while not self._execute_task_srv_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info(f'[{LOGGING_NAME}] /{namespace}/autonomy/task/execute_srv service not available, waiting again...')

    def execute_task(self):
        req = ExecuteTask.Request()
        req.task_id = TASK_ID
        self.future = self._execute_task_srv_client.call_async(req)
        self.future.add_done_callback(self.execute_task_srv_response_cb)

    def execute_task_srv_response_cb(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'[{LOGGING_NAME}] Task executed successfully!')
            else:
                self.get_logger().error(f'[{LOGGING_NAME}] Failed to execute task')
        except Exception as e:
            self.get_logger().error(f'[{LOGGING_NAME}] Service call failed: {e}')

        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    robot_config = read_yaml(ROBOT_CONFIG_PATH)
    namespace = robot_config['system']['ros2']['namespace']

    execute_task_client = ExecuteTaskSrv(namespace)
    execute_task_client.execute_task()

    rclpy.spin(execute_task_client)


if __name__ == '__main__':
    main()
