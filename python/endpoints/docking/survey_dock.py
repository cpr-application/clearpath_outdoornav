from action_msgs.msg import GoalStatus
from clearpath_config.common.utils.yaml import read_yaml
from clearpath_dock_msgs.action import SurveyDock
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

ROBOT_CONFIG_PATH = '/etc/clearpath/robot.yaml'
LOGGING_NAME = 'SurveyDock'

# TODO: Enter your dock info
DOCK_NAME: str = ''
TIMEOUT: float = 30.0


class SurveyDockActionClient(Node):

    def __init__(self, namespace: str):
        super().__init__('survey_dock_client')
        self._action_name = f'/{namespace}/docking/dock_localizer/survey_dock'
        self._action_client = ActionClient(self, SurveyDock, self._action_name)

    def send_survey_dock_goal(self, dock_name, timeout):
        goal_msg = SurveyDock.Goal()
        goal_msg.dock_name = dock_name
        goal_msg.timeout = timeout

        while not self._action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().info(f'[{LOGGING_NAME}] {self._action_name} action server not available, waiting again...')

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.survey_dock_feedback_cb)
        self._send_goal_future.add_done_callback(self.survey_dock_goal_response_cb)

    def survey_dock_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'[{LOGGING_NAME}] Survey dock goal rejected (is a survey already in progress?)')
            self.destroy_node()
            rclpy.shutdown()
            return

        self.get_logger().info(f'[{LOGGING_NAME}] Survey dock goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.survey_dock_get_result_cb)

    def survey_dock_feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'[{LOGGING_NAME}] {feedback.message}: {feedback.samples_collected}/{feedback.samples_required} samples',
            throttle_duration_sec=1)

    def survey_dock_get_result_cb(self, future):
        result = future.result().result
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED and result.success:
            self.get_logger().info(f'[{LOGGING_NAME}] Surveyed dock successfully!')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info(f'[{LOGGING_NAME}] Survey dock cancelled')
        else:
            self.get_logger().error(f'[{LOGGING_NAME}] Failed to survey dock: {result.message}')

        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    robot_config = read_yaml(ROBOT_CONFIG_PATH)
    namespace = robot_config['system']['ros2']['namespace']

    survey_dock_client = SurveyDockActionClient(namespace)
    survey_dock_client.send_survey_dock_goal(DOCK_NAME, TIMEOUT)

    rclpy.spin(survey_dock_client)


if __name__ == '__main__':
    main()
