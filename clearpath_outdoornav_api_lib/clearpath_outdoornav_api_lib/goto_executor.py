#!/usr/bin/python3

from action_msgs.msg import GoalStatus  # Import for reference to status constants
from clearpath_navigation_msgs.action import ExecuteGoTo, ExecuteGoToPOI
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

LOGGING_NAME = 'GoToExecutor'


class GoToExecutor(Node):

    def __init__(self, node: Node, namespace: str = ""):
        self.node = node
        goto_action_name = f'/{namespace}/autonomy/goto' if namespace else 'autonomy/goto'
        goto_poi_action_name = f'/{namespace}/autonomy/goto_poi' if namespace else 'autonomy/goto_poi'

        # initialize goto executor
        self._goto_ac = ActionClient(self.node, ExecuteGoTo, goto_action_name)
        self._goto_goal_handle = None
        self._goto_goal_cancelled = False

        # initialize goto poi executor
        self._goto_poi_ac = ActionClient(self.node, ExecuteGoToPOI, goto_poi_action_name)
        self._goto_poi_goal_handle = None
        self._goto_poi_goal_cancelled = False

        self._goto_running = False
        self._goto_completed = False

    ###################################
    # GoTo Action Client Functions #
    ###################################
    def send_goto_goal(self, map_id, waypoint):
        goal_msg = ExecuteGoTo.Goal()

        # populate goal message
        goal_msg.map_id = map_id
        goal_msg.waypoint = waypoint

        if not self._goto_ac.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error(f'[{LOGGING_NAME}] autonomy/goto action server not available!')
            self.node.destroy_node()
            return

        self._send_goal_future = self._goto_ac.send_goal_async(goal_msg, feedback_callback=self.goto_feedback_cb)
        self._send_goal_future.add_done_callback(self.goto_goal_response_cb)

    def goto_goal_response_cb(self, future):
        self._goto_goal_handle = future.result()
        if not self._goto_goal_handle.accepted:
            self.node.get_logger().info(f'[{LOGGING_NAME}] GoTo Goal rejected')
            self._goto_goal_handle = None
            return

        self._goto_running = True
        self._goto_completed = False
        self._goto_goal_cancelled = False
        self.node.get_logger().info(f'[{LOGGING_NAME}] GoTo Goal accepted')

        self._get_result_future = self._goto_goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.goto_get_result_cb)

    def goto_feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.node.get_logger().info(f'[{LOGGING_NAME}] GoTo has been running for: {feedback.elapsed_time:.2f}s', throttle_duration_sec=60)

    def goto_cancel_response_cb(self, future):
        cancel_response = future.result()
        self.node.get_logger().info(f'[{LOGGING_NAME}] Cancel response received: {cancel_response.return_code}')
        if len(cancel_response.goals_canceling) > 0:
            self.node.get_logger().info(f'[{LOGGING_NAME}] Canceling of goto goal complete')
        else:
            self.node.get_logger().warning(f'[{LOGGING_NAME}] GoTo Goal failed to cancel')

    def goto_get_result_cb(self, future):
        result = future.result().result
        status = future.result().status

        # Compare the status integer against the constants defined in the message
        if status == GoalStatus.STATUS_SUCCEEDED:
            if result.success:
                self.node.get_logger().info(f'[{LOGGING_NAME}] [GoTo Goal Result] Succeeded! Result: Robot completed goto!')
                self._goto_completed = True
            else:
                self.node.get_logger().info(f'[{LOGGING_NAME}] [GoTo Goal Result] Succeeded! Result: Robot failed to complete goto')
                self.node.destroy_node()
                rclpy.shutdown()

        elif status == GoalStatus.STATUS_CANCELED:
            self.node.get_logger().info(f'[{LOGGING_NAME}] [GoTo Goal Result] Cancelled!')
            self._goto_goal_cancelled = True

        elif status == GoalStatus.STATUS_ABORTED:
            self.node.get_logger().info(f'[{LOGGING_NAME}] [GoTo Goal Result] Aborted!')
        else:
            self.node.get_logger().info(f'[{LOGGING_NAME}] [GoTo Goal Result] Finished with unknown status: {status}')

        self._goto_goal_handle = None
        self._goto_running = False

    #############################################
    # GoTo POI Action Client Functions #
    #############################################
    def send_goto_poi_goal(self, map_id, poi_id):
        # populate goal message
        goal_msg = ExecuteGoToPOI.Goal()
        goal_msg.map_uuid = map_id
        goal_msg.poi_uuid = poi_id

        if not self._goto_poi_ac.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error(f'[{LOGGING_NAME}] autonomy/goto_poi action server not available!')
            self.node.destroy_node()
            return

        self._send_goal_future = self._goto_poi_ac.send_goal_async(goal_msg, feedback_callback=self.goto_poi_feedback_cb)
        self._send_goal_future.add_done_callback(self.goto_poi_goal_response_cb)

    def goto_poi_goal_response_cb(self, future):
        self._goto_poi_goal_handle = future.result()
        if not self._goto_poi_goal_handle.accepted:
            self.node.get_logger().info(f'[{LOGGING_NAME}] GoTo POI Goal rejected')
            self._goto_poi_goal_handle = None
            return

        self._goto_running = True
        self._goto_completed = False
        self._goto_goal_cancelled = False
        self.node.get_logger().info(f'[{LOGGING_NAME}] GoTo POI Goal accepted')

        self._get_result_future = self._goto_poi_goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.goto_poi_get_result_cb)

    def goto_poi_feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.node.get_logger().info(f'[{LOGGING_NAME}] GoTo POI has been running for: {feedback.elapsed_time:.2f}s', throttle_duration_sec=60)

    def goto_poi_cancel_response_cb(self, future):
        cancel_response = future.result()
        self.node.get_logger().info(f'[{LOGGING_NAME}] Cancel response received: {cancel_response.return_code}')
        if len(cancel_response.goals_canceling) > 0:
            self.node.get_logger().info(f'[{LOGGING_NAME}] Canceling of goto poi goal complete')
        else:
            self.node.get_logger().warning(f'[{LOGGING_NAME}] GoTo POI Goal failed to cancel')

    def goto_poi_get_result_cb(self, future):
        result = future.result().result
        status = future.result().status

        # Compare the status integer against the constants defined in the message
        if status == GoalStatus.STATUS_SUCCEEDED:
            if result.success:
                self.node.get_logger().info(f'[{LOGGING_NAME}] [GoTo POI Goal Result] Succeeded! Result: Robot completed goto!')
                self._goto_completed = True
            else:
                self.node.get_logger().info(f'[{LOGGING_NAME}] [GoTo POI Goal Result] Succeeded! Result: Robot failed to complete goto')
                self.node.destroy_node()
                rclpy.shutdown()

        elif status == GoalStatus.STATUS_CANCELED:
            self.node.get_logger().info(f'[{LOGGING_NAME}] [GoTo POI Goal Result] Cancelled!')
            self._goto_goal_cancelled = True

        elif status == GoalStatus.STATUS_ABORTED:
            self.node.get_logger().info(f'[{LOGGING_NAME}] [GoTo POI Goal Result] Aborted!')
        else:
            self.node.get_logger().info(f'[{LOGGING_NAME}] [GoTo POI Goal Result] Finished with unknown status: {status}')

        self._goto_goal_handle = None
        self._goto_running = False
