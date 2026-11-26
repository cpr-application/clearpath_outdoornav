# BETA: The go home API is in beta and will be available in the next release.

from action_msgs.msg import GoalStatus  # Import for reference to status constants
from clearpath_config.common.utils.yaml import read_yaml
from clearpath_navigation_msgs.action import ExecuteGoHome
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

ROBOT_CONFIG_PATH = '/etc/clearpath/robot.yaml'
LOGGING_NAME = 'ExecuteGoHome'

# TODO: Enter the uuid of the map to navigate home on
# The home location itself is set with the set_home endpoint
MAP_ID: str = ''


class ExecuteGoHomeActionClient(Node):

    def __init__(self, namespace: str):
        super().__init__('go_home_action_client')
        self._namespace = namespace
        self._action_client = ActionClient(self, ExecuteGoHome, f'/{self._namespace}/autonomy/go_home')
        self._go_home_goal_handle = None
        self._go_home_goal_cancelled = False
        self._go_home_running = False
        self._go_home_completed = False

    #############################################
    # Go Home Action Client Functions #
    #############################################
    def send_go_home_goal(self, map_id):
        # populate goal message
        goal_msg = ExecuteGoHome.Goal()
        goal_msg.map_uuid = map_id

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'[{LOGGING_NAME}] /{self._namespace}/autonomy/go_home action server not available!')
            self.destroy_node()
            rclpy.shutdown()
            return

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.go_home_feedback_cb)
        self._send_goal_future.add_done_callback(self.go_home_goal_response_cb)

    def go_home_goal_response_cb(self, future):
        self._go_home_goal_handle = future.result()
        if not self._go_home_goal_handle.accepted:
            self.get_logger().info(f'[{LOGGING_NAME}] Go Home Goal rejected')
            self._go_home_goal_handle = None
            self.destroy_node()
            rclpy.shutdown()
            return

        self._go_home_running = True
        self._go_home_completed = False
        self._go_home_goal_cancelled = False
        self.get_logger().info(f'[{LOGGING_NAME}] Go Home Goal accepted')

        self._get_result_future = self._go_home_goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.go_home_get_result_cb)

    def go_home_feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'[{LOGGING_NAME}] Go Home has been running for: {feedback.elapsed_time:.2f}s', throttle_duration_sec=60)

    def go_home_get_result_cb(self, future):
        result = future.result().result
        status = future.result().status

        # Compare the status integer against the constants defined in the message
        if status == GoalStatus.STATUS_SUCCEEDED:
            if result.success:
                self.get_logger().info(f'[{LOGGING_NAME}] [Go Home Goal Result] Succeeded! Result: Robot is home!')
                self._go_home_completed = True
            else:
                self.get_logger().info(f'[{LOGGING_NAME}] [Go Home Goal Result] Robot failed to go home: {result.error_msg}')

        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info(f'[{LOGGING_NAME}] [Go Home Goal Result] Cancelled!')
            self._go_home_goal_cancelled = True

        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().info(f'[{LOGGING_NAME}] [Go Home Goal Result] Aborted!')
        else:
            self.get_logger().info(f'[{LOGGING_NAME}] [Go Home Goal Result] Finished with unknown status: {status}')

        self._go_home_goal_handle = None
        self._go_home_running = False

        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    robot_config = read_yaml(ROBOT_CONFIG_PATH)
    namespace = robot_config['system']['ros2']['namespace']

    go_home = ExecuteGoHomeActionClient(namespace)
    go_home.send_go_home_goal(MAP_ID)
    rclpy.spin(go_home)


if __name__ == '__main__':
    main()
