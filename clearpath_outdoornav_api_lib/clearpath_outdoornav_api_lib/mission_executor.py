#!/usr/bin/python3

from action_msgs.msg import GoalStatus  # Import for reference to status constants
from clearpath_navigation_msgs.action import ExecuteMission, ExecuteMissionFromGoal
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

LOGGING_NAME = 'MissionExecutor'


class MissionExecutor(Node):

    def __init__(self, node: Node, namespace: str = ""):
        self.node = node
        mission_action_name = f'/{namespace}/autonomy/mission' if namespace else 'autonomy/mission'
        mission_from_goal_action_name = f'/{namespace}/autonomy/mission_from_goal' if namespace else 'autonomy/mission_from_goal'

        # initialize mission executor
        self._mission_ac = ActionClient(self.node, ExecuteMission, mission_action_name)
        self._mission_goal_handle = None
        self._mission_goal_cancelled = False

        # initialize mission executor
        self._mission_from_goal_ac = ActionClient(self.node, ExecuteMissionFromGoal, mission_from_goal_action_name)
        self._mission_from_goal_goal_handle = None
        self._mission_from_goal_goal_cancelled = False

        self._mission_running = False
        self._mission_completed = False

    ###################################
    # Mission Action Client Functions #
    ###################################
    def send_mission_goal(self, mission_id, map_id):
        goal_msg = ExecuteMission.Goal()

        # populate goal message
        goal_msg.mission_uuid = mission_id
        goal_msg.map_uuid = map_id

        if not self._mission_ac.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error(f'[{LOGGING_NAME}] autonomy/mission action server not available!')
            self.node.destroy_node()
            return

        self._send_goal_future = self._mission_ac.send_goal_async(goal_msg, feedback_callback=self.mission_feedback_cb)
        self._send_goal_future.add_done_callback(self.mission_goal_response_cb)

    def mission_goal_response_cb(self, future):
        self._mission_goal_handle = future.result()
        if not self._mission_goal_handle.accepted:
            self.node.get_logger().info(f'[{LOGGING_NAME}] Mission Goal rejected')
            self._mission_goal_handle = None
            return

        self._mission_running = True
        self._mission_completed = False
        self._mission_goal_cancelled = False
        self.node.get_logger().info(f'[{LOGGING_NAME}] Mission Goal accepted')

        self._get_result_future = self._mission_goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.mission_get_result_cb)

    def mission_feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.node.get_logger().info(f'[{LOGGING_NAME}] Mission has been running for: {feedback.elapsed_time:.2f}s', throttle_duration_sec=60)

    def mission_cancel_response_cb(self, future):
        cancel_response = future.result()
        self.node.get_logger().info(f'[{LOGGING_NAME}] Cancel response received: {cancel_response.return_code}')
        if len(cancel_response.goals_canceling) > 0:
            self.node.get_logger().info(f'[{LOGGING_NAME}] Canceling of mission goal complete')
        else:
            self.node.get_logger().warning(f'[{LOGGING_NAME}] Mission Goal failed to cancel')

    def mission_get_result_cb(self, future):
        result = future.result().result
        status = future.result().status

        # Compare the status integer against the constants defined in the message
        if status == GoalStatus.STATUS_SUCCEEDED:
            if result.success:
                self.node.get_logger().info(f'[{LOGGING_NAME}] [Mission Goal Result] Succeeded! Result: Robot completed mission!')
                self._mission_completed = True
            else:
                self.node.get_logger().info(f'[{LOGGING_NAME}] [Mission Goal Result] Succeeded! Result: Robot failed to complete mission')
                self.node.destroy_node()
                rclpy.shutdown()

        elif status == GoalStatus.STATUS_CANCELED:
            self.node.get_logger().info(f'[{LOGGING_NAME}] [Mission Goal Result] Cancelled!')
            self._mission_goal_cancelled = True

        elif status == GoalStatus.STATUS_ABORTED:
            self.node.get_logger().info(f'[{LOGGING_NAME}] [Mission Goal Result] Aborted!')
        else:
            self.node.get_logger().info(f'[{LOGGING_NAME}] [Mission Goal Result] Finished with unknown status: {status}')

        self._mission_goal_handle = None
        self._mission_running = False

    #############################################
    # Mission From Goal Action Client Functions #
    #############################################
    def send_mission_from_goal_goal(self, mission_id, map_id, goal_id, run_on_start_tasks=True):
        goal_msg = ExecuteMissionFromGoal.Goal()

        # populate goal message
        goal_msg.mission_uuid = mission_id
        goal_msg.map_uuid = map_id
        goal_msg.goal_uuid = goal_id
        goal_msg.run_on_start_tasks = run_on_start_tasks

        if not self._mission_from_goal_ac.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error(f'[{LOGGING_NAME}] autonomy/mission_from_goal action server not available!')
            self.node.destroy_node()
            return

        self._send_goal_future = self._mission_from_goal_ac.send_goal_async(goal_msg, feedback_callback=self.mission_from_goal_feedback_cb)
        self._send_goal_future.add_done_callback(self.mission_from_goal_goal_response_cb)

    def mission_from_goal_goal_response_cb(self, future):
        self._mission_from_goal_goal_handle = future.result()
        if not self._mission_from_goal_goal_handle.accepted:
            self.node.get_logger().info(f'[{LOGGING_NAME}] Mission Goal rejected')
            self._mission_from_goal_goal_handle = None
            return

        self._mission_running = True
        self._mission_completed = False
        self._mission_goal_cancelled = False
        self.node.get_logger().info(f'[{LOGGING_NAME}] Mission Goal accepted')

        self._get_result_future = self._mission_from_goal_goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.mission_from_goal_get_result_cb)

    def mission_from_goal_feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.node.get_logger().info(f'[{LOGGING_NAME}] Mission has been running for: {feedback.elapsed_time:.2f}s', throttle_duration_sec=60)

    def mission_from_goal_cancel_response_cb(self, future):
        cancel_response = future.result()
        self.node.get_logger().info(f'[{LOGGING_NAME}] Cancel response received: {cancel_response.return_code}')
        if len(cancel_response.goals_canceling) > 0:
            self.node.get_logger().info(f'[{LOGGING_NAME}] Canceling of mission goal complete')
        else:
            self.node.get_logger().warning(f'[{LOGGING_NAME}] Mission from goal Goal failed to cancel')

    def mission_from_goal_get_result_cb(self, future):
        result = future.result().result
        status = future.result().status

        # Compare the status integer against the constants defined in the message
        if status == GoalStatus.STATUS_SUCCEEDED:
            if result.success:
                self.node.get_logger().info(f'[{LOGGING_NAME}] [Mission from goal Goal Result] Succeeded! Result: Robot completed mission!')
                self._mission_completed = True
            else:
                self.node.get_logger().info(f'[{LOGGING_NAME}] [Mission from goal Goal Result] Succeeded! Result: Robot failed to complete mission')
                self.node.destroy_node()
                rclpy.shutdown()

        elif status == GoalStatus.STATUS_CANCELED:
            self.node.get_logger().info(f'[{LOGGING_NAME}] [Mission from goal Goal Result] Cancelled!')
            self._mission_goal_cancelled = True

        elif status == GoalStatus.STATUS_ABORTED:
            self.node.get_logger().info(f'[{LOGGING_NAME}] [Mission from goal Goal Result] Aborted!')
        else:
            self.node.get_logger().info(f'[{LOGGING_NAME}] [Mission from goal Goal Result] Finished with unknown status: {status}')

        self._mission_goal_handle = None
        self._mission_running = False
