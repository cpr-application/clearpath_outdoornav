from action_msgs.msg import GoalStatus  # Import for reference to status constants
from clearpath_config.common.utils.yaml import read_yaml
from clearpath_dock_msgs.action import Dock
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

ROBOT_CONFIG_PATH = '/etc/clearpath/robot.yaml'
LOGGING_NAME = 'Dock'

# TODO: Enter your dock name
DOCK_NAME: str = ''


class DockActionClient(Node):

    def __init__(self, namespace: str):
        super().__init__('execute_dock')
        self._namespace = namespace
        self._action_client = ActionClient(self, Dock, f'/{self._namespace}/autonomy/dock_local')
        self._dock_goal_handle = None

        self._dock_running = False

    ###################################
    # Dock Action Client Functions #
    ###################################
    def send_dock_goal(self, dock_name):
        goal_msg = Dock.Goal()
        goal_msg.dock_name = dock_name

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'[{LOGGING_NAME}] /{self._namespace}/autonomy/dock_local action server not available!')
            self.destroy_node()
            rclpy.shutdown()
            return

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.dock_feedback_cb)
        self._send_goal_future.add_done_callback(self.dock_goal_response_cb)

    def dock_goal_response_cb(self, future):
        self._dock_goal_handle = future.result()
        if not self._dock_goal_handle.accepted:
            self.get_logger().info(f'[{LOGGING_NAME}] Dock goal rejected')
            self.destroy_node()
            rclpy.shutdown()
            return

        self._dock_running = True
        self.get_logger().info(f'[{LOGGING_NAME}] Undock goal accepted')

        self._get_result_future = self._dock_goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.dock_get_result_cb)

    def dock_feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'[{LOGGING_NAME}] Docking has been running for: {feedback.elapsed_time:.2f}s', throttle_duration_sec=5)

    def dock_cancel_response_cb(self, future):
        cancel_response = future.result()
        self.get_logger().info(f'[{LOGGING_NAME}] Cancel response received: {cancel_response.return_code}')
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info(f'[{LOGGING_NAME}] Cancelling of undock goal complete')
        else:
            self.get_logger().warning(f'[{LOGGING_NAME}] Undock goal failed to cancel')

    def dock_get_result_cb(self, future):
        result = future.result().result
        status = future.result().status

        # Compare the status integer against the constants defined in the message
        if status == GoalStatus.STATUS_SUCCEEDED:
            if result.success:
                self.get_logger().info(f'[{LOGGING_NAME}] [Dock Goal Result] Succeeded! Result: Robot docked successfully!')
            else:
                self.get_logger().info(f'[{LOGGING_NAME}] [Dock Goal Result] Succeeded! Result: Robot failed to dock')

        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info(f'[{LOGGING_NAME}] [Dock Goal Result] Cancelled!')

        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().info(f'[{LOGGING_NAME}] [Dock Goal Result] Aborted!')
        else:
            self.get_logger().info(f'[{LOGGING_NAME}] [Dock Goal Result] Finished with unknown status: {status}')

        self._dock_goal_handle = None
        self._dock_running = False

        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    robot_config = read_yaml(ROBOT_CONFIG_PATH)
    namespace = robot_config['system']['ros2']['namespace']

    dock_action_client = DockActionClient(namespace)
    dock_action_client.send_dock_goal(DOCK_NAME)
    rclpy.spin(dock_action_client)


if __name__ == '__main__':
    main()
