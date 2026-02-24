from action_msgs.msg import GoalStatus  # Import for reference to status constants
from clearpath_config.common.utils.yaml import read_yaml
from clearpath_navigation_msgs.action import ExecuteGoToPOI
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

ROBOT_CONFIG_PATH = '/etc/clearpath/robot.yaml'
LOGGING_NAME = 'ExecuteGoToPOI'

# TODO: Enter your map uuid and point of interest uuid
MAP_ID: str = ''
POI_ID: str = ''


class ExecuteGoToPOIActionClient(Node):

    def __init__(self, namespace: str):
        super().__init__('goto_poi_action_client')
        self._namespace = namespace
        self._action_client = ActionClient(self, ExecuteGoToPOI, f'/{self._namespace}/autonomy/goto_poi')
        self._goto_goal_handle = None
        self._goto_goal_cancelled = False
        self._goto_running = False
        self._goto_completed = False

    #############################################
    # GoTo POI Action Client Functions #
    #############################################
    def send_goto_poi_goal(self, map_id, poi_id):
        # populate goal message
        goal_msg = ExecuteGoToPOI.Goal()
        goal_msg.map_uuid = map_id
        goal_msg.poi_uuid = poi_id

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'[{LOGGING_NAME}] /{self._namespace}/autonomy/goto_poi action server not available!')
            self.destroy_node()
            rclpy.shutdown()
            return

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.goto_poi_feedback_cb)
        self._send_goal_future.add_done_callback(self.goto_poi_goal_response_cb)

    def goto_poi_goal_response_cb(self, future):
        self._goto_poi_goal_handle = future.result()
        if not self._goto_poi_goal_handle.accepted:
            self.get_logger().info(f'[{LOGGING_NAME}] GoTo POI Goal rejected')
            self._goto_poi_goal_handle = None
            self.destroy_node()
            rclpy.shutdown()
            return

        self._goto_running = True
        self._goto_completed = False
        self._goto_goal_cancelled = False
        self.get_logger().info(f'[{LOGGING_NAME}] GoTo POI Goal accepted')

        self._get_result_future = self._goto_poi_goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.goto_poi_get_result_cb)

    def goto_poi_feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'[{LOGGING_NAME}] GoTo POI has been running for: {feedback.elapsed_time:.2f}s', throttle_duration_sec=60)

    def goto_poi_cancel_response_cb(self, future):
        cancel_response = future.result()
        self.get_logger().info(f'[{LOGGING_NAME}] Cancel response received: {cancel_response.return_code}')
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info(f'[{LOGGING_NAME}] Canceling of goto poi goal complete')
        else:
            self.get_logger().warning(f'[{LOGGING_NAME}] GoTo POI Goal failed to cancel')

    def goto_poi_get_result_cb(self, future):
        result = future.result().result
        status = future.result().status

        # Compare the status integer against the constants defined in the message
        if status == GoalStatus.STATUS_SUCCEEDED:
            if result.success:
                self.get_logger().info(f'[{LOGGING_NAME}] [GoTo POI Goal Result] Succeeded! Result: Robot completed goto!')
                self._goto_completed = True
            else:
                self.get_logger().info(f'[{LOGGING_NAME}] [GoTo POI Goal Result] Succeeded! Result: Robot failed to complete goto')

        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info(f'[{LOGGING_NAME}] [GoTo POI Goal Result] Cancelled!')
            self._goto_goal_cancelled = True

        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().info(f'[{LOGGING_NAME}] [GoTo POI Goal Result] Aborted!')
        else:
            self.get_logger().info(f'[{LOGGING_NAME}] [GoTo POI Goal Result] Finished with unknown status: {status}')

        self._goto_goal_handle = None
        self._goto_running = False

        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    robot_config = read_yaml(ROBOT_CONFIG_PATH)
    namespace = robot_config['system']['ros2']['namespace']

    goto = ExecuteGoToPOIActionClient(namespace)
    goto.send_goto_poi_goal(MAP_ID, POI_ID)
    rclpy.spin(goto)


if __name__ == '__main__':
    main()
