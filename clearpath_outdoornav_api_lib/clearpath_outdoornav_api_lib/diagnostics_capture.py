#!/usr/bin/python3

from action_msgs.msg import GoalStatus  # Import for reference to status constants
from clearpath_diagnostics_msgs.action import CaptureDiagnostics
from rclpy.action import ActionClient
from rclpy.node import Node

LOGGING_NAME = 'DiagnosticsCapture'

PHASE_NAMES = {
    CaptureDiagnostics.Feedback.PHASE_DISCOVERING: 'discovering',
    CaptureDiagnostics.Feedback.PHASE_MONITORING: 'monitoring',
    CaptureDiagnostics.Feedback.PHASE_COLLECTING: 'collecting',
    CaptureDiagnostics.Feedback.PHASE_FINALIZING: 'finalizing',
}


class DiagnosticsCapture:
    """Starts and cancels OutdoorNav diagnostics captures through outdoornav/diagnostics."""

    def __init__(self, node: Node, namespace: str = ''):
        self.node = node
        capture_action_name = (f'/{namespace}/outdoornav/diagnostics/capture' if namespace
                               else 'outdoornav/diagnostics/capture')
        self._capture_ac = ActionClient(self.node, CaptureDiagnostics, capture_action_name)
        self._capture_goal_handle = None
        self._capture_running = False
        self._capture_feedback = None
        self._capture_result = None
        self._capture_status = None

    def start_capture(self, name: str = '', duration: float = 0.0,
                      include_heavy_topics: bool = False, record_bag: bool = False) -> bool:
        """
        Request a capture. Returns False if the action server is unavailable.

        An empty name uses nav_diag_<namespace>_<timestamp>, and a duration of 0 uses the
        server's default monitoring window.
        """
        goal_msg = CaptureDiagnostics.Goal()
        goal_msg.name = name
        goal_msg.duration = float(duration)
        goal_msg.include_heavy_topics = include_heavy_topics
        goal_msg.record_bag = record_bag

        if not self._capture_ac.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error(
                f'[{LOGGING_NAME}] outdoornav/diagnostics/capture action server not available!')
            return False

        self._capture_feedback = None
        self._capture_result = None
        self._capture_status = None
        self._send_goal_future = self._capture_ac.send_goal_async(
            goal_msg, feedback_callback=self.capture_feedback_cb)
        self._send_goal_future.add_done_callback(self.capture_goal_response_cb)
        return True

    def cancel_capture(self):
        """End the monitoring window early, or abort collection if it has already ended."""
        if self._capture_goal_handle is None:
            self.node.get_logger().warning(f'[{LOGGING_NAME}] No capture is running')
            return
        cancel_future = self._capture_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self.capture_cancel_response_cb)

    def is_running(self) -> bool:
        return self._capture_running

    def get_feedback(self):
        """Return the latest CaptureDiagnostics.Feedback, or None."""
        return self._capture_feedback

    def get_result(self):
        """Return the last CaptureDiagnostics.Result, or None while a capture is running."""
        return self._capture_result

    def get_status(self):
        """Return the last capture's action_msgs/GoalStatus code, or None."""
        return self._capture_status

    def capture_goal_response_cb(self, future):
        self._capture_goal_handle = future.result()
        if not self._capture_goal_handle.accepted:
            self.node.get_logger().error(
                f'[{LOGGING_NAME}] Capture Goal rejected (is another capture running?)')
            self._capture_goal_handle = None
            return

        self._capture_running = True
        self.node.get_logger().info(f'[{LOGGING_NAME}] Capture Goal accepted')
        self._get_result_future = self._capture_goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.capture_get_result_cb)

    def capture_feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        self._capture_feedback = feedback
        self.node.get_logger().info(
            f'[{LOGGING_NAME}] {PHASE_NAMES.get(feedback.phase, feedback.phase)} '
            f'({feedback.progress * 100:.0f}% of window, {feedback.time_elapsed:.0f}s): '
            f'{feedback.message}', throttle_duration_sec=10)

    def capture_cancel_response_cb(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.node.get_logger().info(f'[{LOGGING_NAME}] Capture cancel accepted')
        else:
            self.node.get_logger().warning(f'[{LOGGING_NAME}] Capture Goal failed to cancel')

    def capture_get_result_cb(self, future):
        result = future.result().result
        status = future.result().status
        self._capture_result = result
        self._capture_status = status

        if status in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED) and result.success:
            self.node.get_logger().info(f'[{LOGGING_NAME}] [Capture Result] {result.message}')
            if result.has_error_findings:
                self.node.get_logger().warning(
                    f'[{LOGGING_NAME}] [Capture Result] The summary reports ERROR findings')
        elif status == GoalStatus.STATUS_CANCELED:
            self.node.get_logger().info(
                f'[{LOGGING_NAME}] [Capture Result] Cancelled: {result.message}')
        else:
            self.node.get_logger().error(
                f'[{LOGGING_NAME}] [Capture Result] Failed: {result.message}')

        self._capture_goal_handle = None
        self._capture_running = False
