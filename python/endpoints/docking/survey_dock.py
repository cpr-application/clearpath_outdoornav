from clearpath_config.common.utils.yaml import read_yaml
from clearpath_dock_msgs.srv import SurveyDock
import rclpy
from rclpy.node import Node

ROBOT_CONFIG_PATH = '/etc/clearpath/robot.yaml'
LOGGING_NAME = 'SurveyDock'

# TODO: Enter your dock info
DOCK_NAME: str = ''
TIMEOUT: float = 30.0


class SurveyDockCLient(Node):

    def __init__(self, namespace: str):
        super().__init__('add_dock_client')
        self._survey_dock_client = self.create_client(SurveyDock, f'/{namespace}/docking/dock_localizer/survey_dock')

        while not self._survey_dock_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info(f'[{LOGGING_NAME}] /{namespace}/docking/dock_localizer/survey_dock service not available, waiting again...')

    def survey_dock(self, dock_name, timeout):
        req = SurveyDock.Request()
        req.dock_name = dock_name
        req.timeout = timeout
        self.future = self._survey_dock_client.call_async(req)
        self.future.add_done_callback(self.survey_dock_srv_response_cb)

    def survey_dock_srv_response_cb(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'[{LOGGING_NAME}] Surveyed dock successfully!')
            else:
                self.get_logger().error(f'[{LOGGING_NAME}] Failed to survey dock')
        except Exception as e:
            self.get_logger().error(f'[{LOGGING_NAME}] Service call failed: {e}')

        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    robot_config = read_yaml(ROBOT_CONFIG_PATH)
    namespace = robot_config['system']['ros2']['namespace']

    survey_dock_client = SurveyDockCLient(namespace)
    survey_dock_client.survey_dock(DOCK_NAME, TIMEOUT)

    rclpy.spin(survey_dock_client)


if __name__ == '__main__':
    main()
