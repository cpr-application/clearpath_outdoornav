from action_msgs.msg import GoalStatus  # Import for reference to status constants
from clearpath_config.common.utils.yaml import read_yaml
from clearpath_dock_msgs.action import Undock
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

ROBOT_CONFIG_PATH = '/etc/clearpath/robot.yaml'
LOGGING_NAME = 'Undock'

# TODO: Enter your dock name
DOCK_NAME: str = ''


class UndockActionClient(Node):

    def __init__(self, namespace: str):
        super().__init__('execute_undock')
        self._namespace = namespace
        self._action_client = ActionClient(self, Undock, f'/{self._namespace}/autonomy/undock')
        self._undock_goal_handle = None

        self._map_dock_running = False
        self._undock_running = False
        self._docked = False

    ###################################
    # Undock Action Client Functions #
    ###################################
    def send_undock_goal(self, dock_name):
        goal_msg = Undock.Goal()
        goal_msg.dock_name = dock_name

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'[{LOGGING_NAME}] /{self._namespace}/autonomy/undock action server not available!')
            self.destroy_node()
            rclpy.shutdown()
            return

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.undock_feedback_cb)
        self._send_goal_future.add_done_callback(self.undock_goal_response_cb)

    def undock_goal_response_cb(self, future):
        self._undock_goal_handle = future.result()
        if not self._undock_goal_handle.accepted:
            self.get_logger().info(f'[{LOGGING_NAME}] Undock goal rejected')
            self.destroy_node()
            rclpy.shutdown()
            return

        self._undock_running = True
        self.get_logger().info(f'[{LOGGING_NAME}] Undock goal accepted')

        self._get_result_future = self._undock_goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.undock_get_result_cb)

    def undock_feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'[{LOGGING_NAME}] Undocking has been running for: {feedback.elapsed_time:.2f}s', throttle_duration_sec=5)

    def undock_cancel_response_cb(self, future):
        cancel_response = future.result()
        self.get_logger().info(f'[{LOGGING_NAME}] Cancel response received: {cancel_response.return_code}')
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info(f'[{LOGGING_NAME}] Cancelling of undock goal complete')
        else:
            self.get_logger().warning(f'[{LOGGING_NAME}] Undock goal failed to cancel')

    def undock_get_result_cb(self, future):
        result = future.result().result
        status = future.result().status

        self._docked = True

        # Compare the status integer against the constants defined in the message
        if status == GoalStatus.STATUS_SUCCEEDED:
            if result.success:
                self.get_logger().info(f'[{LOGGING_NAME}] [Undock Goal Result] Succeeded! Result: Robot undocked successfully!')
                self._docked = False
            else:
                self.get_logger().info(f'[{LOGGING_NAME}] [Undock Goal Result] Succeeded! Result: Robot failed to undock')

        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info(f'[{LOGGING_NAME}] [Undock Goal Result] Cancelled!')

        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().info(f'[{LOGGING_NAME}] [Undock Goal Result] Aborted!')
            self._docked = False
        else:
            self.get_logger().info(f'[{LOGGING_NAME}] [Undock Goal Result] Finished with unknown status: {status}')

        self._undock_goal_handle = None
        self._undock_running = False

        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    robot_config = read_yaml(ROBOT_CONFIG_PATH)
    namespace = robot_config['system']['ros2']['namespace']

    undock = UndockActionClient(namespace)
    undock.send_undock_goal(DOCK_NAME)
    rclpy.spin(undock)


if __name__ == '__main__':
    main()
