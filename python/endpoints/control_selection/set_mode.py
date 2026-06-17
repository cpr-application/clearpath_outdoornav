from clearpath_config.common.utils.yaml import read_yaml
from clearpath_control_selection_msgs.srv import SetControlMode
from clearpath_control_selection_msgs.msg import ControlMode
import rclpy
from rclpy.node import Node

ROBOT_CONFIG_PATH = '/etc/clearpath/robot.yaml'
LOGGING_NAME = 'SetControlMode'

# TODO: Enter your control mode
CONTROL_MODE: int = ControlMode.MANUAL # options: MANUAL, AUTONOMOUS, NEUTRAL. See clearpath_control_selection_msgs/msg/ControlMode.msg for more details.


class SetControlModeClient(Node):

    def __init__(self, namespace: str):
        super().__init__('set_control_mode_client')
        self._set_control_mode_client = self.create_client(SetControlMode, f'/{namespace}/control_selection/set_mode')

        while not self._set_control_mode_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info(f'[{LOGGING_NAME}] /{namespace}/control_selection/set_mode service not available, waiting again...')

    def set_control_mode(self, mode):
        req = SetControlMode.Request()
        req.mode.mode = mode
        self.future = self._set_control_mode_client.call_async(req)
        self.future.add_done_callback(self.set_control_mode_srv_response_cb)

    def set_control_mode_srv_response_cb(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'[{LOGGING_NAME}] Control mode set successfully!')
            else:
                self.get_logger().error(f'[{LOGGING_NAME}] Failed to set control mode')
        except Exception as e:
            self.get_logger().error(f'[{LOGGING_NAME}] Service call failed: {e}')

        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    robot_config = read_yaml(ROBOT_CONFIG_PATH)
    namespace = robot_config['system']['ros2']['namespace']

    set_control_mode_client = SetControlModeClient(namespace)
    set_control_mode_client.set_control_mode(CONTROL_MODE)

    rclpy.spin(set_control_mode_client)


if __name__ == '__main__':
    main()
