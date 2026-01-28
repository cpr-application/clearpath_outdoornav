from clearpath_config.common.utils.yaml import read_yaml
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

ROBOT_CONFIG_PATH = '/etc/clearpath/robot.yaml'
LOGGING_NAME = 'PauseAutonomy'


class PauseAutonomy(Node):

    def __init__(self, namespace: str):
        super().__init__('pause_autonomy_client')
        self._autonomy_pause_client = self.create_client(SetBool, f'/{namespace}/autonomy/pause')

        while not self._autonomy_pause_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info(f'[{LOGGING_NAME}] /{namespace}/autonomy/pause service not available, waiting again...')

    def pause(self):
        req = SetBool.Request()
        req.data = True
        self.future = self._autonomy_pause_client.call_async(req)
        self.future.add_done_callback(self.autonomy_pause_srv_response_cb)

    def autonomy_pause_srv_response_cb(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'[{LOGGING_NAME}] Autonomy paused!')
            else:
                self.get_logger().error(f'[{LOGGING_NAME}] Failed to pause autonomy')
        except Exception as e:
            self.get_logger().error(f'[{LOGGING_NAME}] Service call failed: {e}')

        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    robot_config = read_yaml(ROBOT_CONFIG_PATH)
    namespace = robot_config['system']['ros2']['namespace']

    pause_autonomy_client = PauseAutonomy(namespace)
    pause_autonomy_client.pause()

    rclpy.spin(pause_autonomy_client)


if __name__ == '__main__':
    main()
