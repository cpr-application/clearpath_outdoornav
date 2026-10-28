from action_msgs.msg import GoalStatus  # Import for reference to status constants
from clearpath_config.common.utils.yaml import read_yaml
from clearpath_diagnostics_msgs.action import CaptureDiagnostics
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

ROBOT_CONFIG_PATH = '/etc/clearpath/robot.yaml'
LOGGING_NAME = 'CaptureDiagnostics'

# TODO: Optionally name the capture (letters, digits, '.', '_' and '-').
# Empty uses onav_diagnostics_<timestamp>.
CAPTURE_NAME: str = ''
# Monitoring window in seconds; 0 uses the server's default (60 s)
DURATION: float = 0.0
INCLUDE_HEAVY_TOPICS: bool = False
RECORD_BAG: bool = False

PHASE_NAMES = {
    CaptureDiagnostics.Feedback.PHASE_DISCOVERING: 'Discovering',
    CaptureDiagnostics.Feedback.PHASE_MONITORING: 'Monitoring',
    CaptureDiagnostics.Feedback.PHASE_COLLECTING: 'Collecting',
    CaptureDiagnostics.Feedback.PHASE_FINALIZING: 'Finalizing',
}


class CaptureDiagnosticsActionClient(Node):

    def __init__(self, namespace: str):
        super().__init__('capture_diagnostics')
        self._namespace = namespace
        self._action_client = ActionClient(
            self, CaptureDiagnostics, f'/{self._namespace}/outdoornav/diagnostics/capture')
        self._goal_handle = None
        self.done = False

    def send_goal(self):
        goal_msg = CaptureDiagnostics.Goal()
        goal_msg.name = CAPTURE_NAME
        goal_msg.duration = DURATION
        goal_msg.include_heavy_topics = INCLUDE_HEAVY_TOPICS
        goal_msg.record_bag = RECORD_BAG

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'[{LOGGING_NAME}] /{self._namespace}/outdoornav/diagnostics/capture action server not available!')
            self.done = True
            return

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.capture_feedback_cb)
        self._send_goal_future.add_done_callback(self.capture_goal_response_cb)

    def cancel(self):
        if self._goal_handle is None:
            self.done = True
            return
        self.get_logger().info(f'[{LOGGING_NAME}] Ending the capture early; waiting for results...')
        self._goal_handle.cancel_goal_async()

    def capture_goal_response_cb(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().error(f'[{LOGGING_NAME}] Capture Goal rejected (is another capture running?)')
            self._goal_handle = None
            self.done = True
            return

        self.get_logger().info(f'[{LOGGING_NAME}] Capture Goal accepted. Press Ctrl-C to end the monitoring window early.')
        self._get_result_future = self._goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.capture_get_result_cb)

    def capture_feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'[{LOGGING_NAME}] {PHASE_NAMES.get(feedback.phase, feedback.phase)} '
            f'({feedback.progress * 100:.0f}%, {feedback.time_elapsed:.0f}s): {feedback.message}',
            throttle_duration_sec=5)

    def capture_get_result_cb(self, future):
        result = future.result().result
        status = future.result().status

        if status in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED) and result.success:
            self.get_logger().info(f'[{LOGGING_NAME}] [Capture Goal Result] {result.message}')
            if result.has_error_findings:
                self.get_logger().warning(f'[{LOGGING_NAME}] [Capture Goal Result] The capture summary reports ERROR findings')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info(f'[{LOGGING_NAME}] [Capture Goal Result] Cancelled: {result.message}')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error(f'[{LOGGING_NAME}] [Capture Goal Result] Aborted: {result.message}')
        else:
            self.get_logger().error(f'[{LOGGING_NAME}] [Capture Goal Result] Finished with unknown status: {status}')

        self._goal_handle = None
        self.done = True


def main(args=None):
    # Handle Ctrl-C ourselves so the capture can be cancelled and its results still received
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    robot_config = read_yaml(ROBOT_CONFIG_PATH)
    namespace = robot_config['system']['ros2']['namespace']

    capture = CaptureDiagnosticsActionClient(namespace)
    capture.send_goal()
    cancelled = False
    while rclpy.ok() and not capture.done:
        try:
            rclpy.spin_once(capture, timeout_sec=0.5)
        except KeyboardInterrupt:
            if cancelled:
                break
            cancelled = True
            capture.cancel()

    capture.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
