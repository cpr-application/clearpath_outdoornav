from action_msgs.msg import GoalStatus  # Import for reference to status constants
from clearpath_config.common.utils.yaml import read_yaml
from clearpath_dock_msgs.action import MapDock
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

ROBOT_CONFIG_PATH = '/etc/clearpath/robot.yaml'
LOGGING_NAME = 'MapDock'

# TODO: Enter your mission and map uuids
DOCK_NAME: str = ''
MAP_ID: str = ''


class MapDockActionClient(Node):

    def __init__(self, namespace: str):
        super().__init__('execute_map_dock')
        self._namespace = namespace
        self._action_client = ActionClient(self, MapDock, f'/{self._namespace}/autonomy/dock_map')

        self._map_dock_goal_handle = None
        self._map_dock_running = False
        self._docked = False

    def send_map_dock_goal(self, dock_name, map_id):
        goal_msg = MapDock.Goal()
        goal_msg.dock_name = dock_name
        goal_msg.map_uuid = map_id

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'[{LOGGING_NAME}] /{self._namespace}/autonomy/dock_map action server not available!')
            self.destroy_node()
            rclpy.shutdown()
            return

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.map_dock_feedback_cb)
        self._send_goal_future.add_done_callback(self.map_dock_goal_response_cb)

    def map_dock_goal_response_cb(self, future):
        self._map_dock_goal_handle = future.result()
        if not self._map_dock_goal_handle.accepted:
            self.get_logger().info(f'[{LOGGING_NAME}] Map Dock goal rejected')
            self.destroy_node()
            rclpy.shutdown()
            return

        self._map_dock_running = True
        self.get_logger().info(f'[{LOGGING_NAME}] Map Dock goal accepted')

        self._get_result_future = self._map_dock_goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.map_dock_get_result_cb)

    def map_dock_feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'[{LOGGING_NAME}] Map docking has been running for: {feedback.elapsed_time:.2f}s', throttle_duration_sec=20)

    def map_dock_get_result_cb(self, future):
        result = future.result().result
        status = future.result().status

        self._docked = False

        # Compare the status integer against the constants defined in the message
        if status == GoalStatus.STATUS_SUCCEEDED:
            if result.success:
                self.get_logger().info(f'[{LOGGING_NAME}] [Map Dock Goal Result] Succeeded! Result: Robot docked successfully!')
                self._docked = True
            else:
                self.get_logger().info(f'[{LOGGING_NAME}] [Map Dock Goal Result] Succeeded! Result: Robot failed to dock')

        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info(f'[{LOGGING_NAME}] [Map Dock Goal Result] Cancelled!')

        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().info(f'[{LOGGING_NAME}] [Map Dock Goal Result] Aborted!')
        else:
            self.get_logger().info(f'[{LOGGING_NAME}] [Map Dock Goal Result] Finished with unknown status: {status}')

        self._map_dock_goal_handle = None
        self._map_dock_running = False

        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    robot_config = read_yaml(ROBOT_CONFIG_PATH)
    namespace = robot_config['system']['ros2']['namespace']

    map_dock = MapDockActionClient(namespace)
    map_dock.send_map_dock_goal(DOCK_NAME, MAP_ID)
    rclpy.spin(map_dock)


if __name__ == '__main__':
    main()
