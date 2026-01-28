from clearpath_config.common.utils.yaml import read_yaml
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

ROBOT_CONFIG_PATH = '/etc/clearpath/robot.yaml'
LOGGING_NAME = 'StopAutonomy'


class StopAutonomy(Node):

    def __init__(self, namespace: str):
        super().__init__('stop_autonomy_client')
        self._autonomy_stop_client = self.create_client(Trigger, f'/{namespace}/autonomy/stop')

        while not self._autonomy_stop_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info(f'[{LOGGING_NAME}] /{namespace}/autonomy/stop service not available, waiting again...')

    def stop(self):
        req = Trigger.Request()
        self.future = self._autonomy_stop_client.call_async(req)
        self.future.add_done_callback(self.autonomy_stop_srv_response_cb)

    def autonomy_stop_srv_response_cb(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'[{LOGGING_NAME}] Autonomy stopped!')
            else:
                self.get_logger().error(f'[{LOGGING_NAME}] Failed to stop autonomy')
        except Exception as e:
            self.get_logger().error(f'[{LOGGING_NAME}] Service call failed: {e}')

        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    robot_config = read_yaml(ROBOT_CONFIG_PATH)
    namespace = robot_config['system']['ros2']['namespace']

    stop_autonomy_client = StopAutonomy(namespace)
    stop_autonomy_client.stop()

    rclpy.spin(stop_autonomy_client)


if __name__ == '__main__':
    main()
