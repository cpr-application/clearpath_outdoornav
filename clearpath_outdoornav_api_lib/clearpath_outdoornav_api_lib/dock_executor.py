#!/usr/bin/python3

from action_msgs.msg import GoalStatus  # Import for reference to status constants
from clearpath_dock_msgs.action import Dock, MapDock, Undock
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

LOGGING_NAME = 'DockExecutor'


class DockExecutor(Node):

    def __init__(self, node: Node, namespace: str = ""):
        self.node = node
        map_dock_action_name = f'/{namespace}/autonomy/dock_map' if namespace else 'autonomy/dock_map'
        local_dock_action_name = f'/{namespace}/autonomy/dock_local' if namespace else 'autonomy/dock_local'
        undock_action_name = f'/{namespace}/autonomy/undock' if namespace else 'autonomy/undock'
        dock_param_srv = f'/{namespace}/docking/get_parameters' if namespace else 'docking/get_parameters'

        # initialize map dock executor
        self._map_dock_ac = ActionClient(self.node, MapDock, map_dock_action_name)
        self._map_dock_goal_handle = None

        # initialize local dock executor
        self._local_dock_ac = ActionClient(self.node, Dock, local_dock_action_name)
        self._local_dock_goal_handle = None

        # initialize undock executor
        self._undock_ac = ActionClient(self.node, Undock, undock_action_name)
        self._undock_goal_handle = None

        self._param_client = self.node.create_client(GetParameters, dock_param_srv)
        self.req = GetParameters.Request()
        while not self._param_client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().info(f'[{LOGGING_NAME}] docking/get_parameters service not available, waiting again...')

        self._map_dock_running = False
        self._local_dock_running = False
        self._undock_running = False
        self._docked = False

    ####################################
    # Map Dock Action Client Functions #
    ####################################
    def send_map_dock_goal(self, dock_name, map_id):
        goal_msg = MapDock.Goal()
        goal_msg.dock_name = dock_name
        goal_msg.map_uuid = map_id

        if not self._map_dock_ac.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error(f'[{LOGGING_NAME}] autonomy/dock_map action server not available!')
            self.destroy_node()
            return

        self._send_goal_future = self._map_dock_ac.send_goal_async(goal_msg, feedback_callback=self.map_dock_feedback_cb)
        self._send_goal_future.add_done_callback(self.map_dock_goal_response_cb)

    def map_dock_goal_response_cb(self, future):
        self._map_dock_goal_handle = future.result()
        if not self._map_dock_goal_handle.accepted:
            self.node.get_logger().info(f'[{LOGGING_NAME}] Map Dock goal rejected')
            return

        self._map_dock_running = True
        self.node.get_logger().info(f'[{LOGGING_NAME}] Map Dock goal accepted')

        self._get_result_future = self._map_dock_goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.map_dock_get_result_cb)

    def map_dock_feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.node.get_logger().info(f'[{LOGGING_NAME}] Map docking has been running for: {feedback.elapsed_time:.2f}s', throttle_duration_sec=20)

    def map_dock_get_result_cb(self, future):
        result = future.result().result
        status = future.result().status

        self._docked = False

        # Compare the status integer against the constants defined in the message
        if status == GoalStatus.STATUS_SUCCEEDED:
            if result.success:
                self.node.get_logger().info(f'[{LOGGING_NAME}] [Map Dock Goal Result] Succeeded! Result: Robot docked successfully!')
                self._docked = True
            else:
                self.node.get_logger().info(f'[{LOGGING_NAME}] [Map Dock Goal Result] Succeeded! Result: Robot failed to dock')
                self.node.destroy_node()
                rclpy.shutdown()

        elif status == GoalStatus.STATUS_CANCELED:
            self.node.get_logger().info(f'[{LOGGING_NAME}] [Map Dock Goal Result] Cancelled!')

        elif status == GoalStatus.STATUS_ABORTED:
            self.node.get_logger().info(f'[{LOGGING_NAME}] [Map Dock Goal Result] Aborted!')
        else:
            self.node.get_logger().info(f'[{LOGGING_NAME}] [Map Dock Goal Result] Finished with unknown status: {status}')

        self._map_dock_goal_handle = None
        self._map_dock_running = False

    ######################################
    # Local Dock Action Client Functions #
    ######################################
    def send_dock_goal(self, dock_name):
        goal_msg = Dock.Goal()
        goal_msg.dock_name = dock_name

        if not self._local_dock_ac.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error(f'[{LOGGING_NAME}] autonomy/dock_local action server not available!')
            self.destroy_node()
            return

        self._send_goal_future = self._local_dock_ac.send_goal_async(goal_msg, feedback_callback=self.local_dock_feedback_cb)
        self._send_goal_future.add_done_callback(self.local_dock_goal_response_cb)

    def local_dock_goal_response_cb(self, future):
        self._local_dock_goal_handle = future.result()
        if not self._local_dock_goal_handle.accepted:
            self.node.get_logger().info(f'[{LOGGING_NAME}] Local Dock goal rejected')
            return

        self._local_dock_running = True
        self.node.get_logger().info(f'[{LOGGING_NAME}] Local Dock goal accepted')

        self._get_result_future = self._local_dock_goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.local_dock_get_result_cb)

    def local_dock_feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.node.get_logger().info(f'[{LOGGING_NAME}] Local docking has been running for: {feedback.elapsed_time:.2f}s', throttle_duration_sec=20)

    def local_dock_get_result_cb(self, future):
        result = future.result().result
        status = future.result().status

        self._docked = False

        # Compare the status integer against the constants defined in the message
        if status == GoalStatus.STATUS_SUCCEEDED:
            if result.success:
                self.node.get_logger().info(f'[{LOGGING_NAME}] [Local Dock Goal Result] Succeeded! Result: Robot docked successfully!')
                self._docked = True
            else:
                self.node.get_logger().info(f'[{LOGGING_NAME}] [Local Dock Goal Result] Succeeded! Result: Robot failed to dock')
                self.node.destroy_node()
                rclpy.shutdown()

        elif status == GoalStatus.STATUS_CANCELED:
            self.node.get_logger().info(f'[{LOGGING_NAME}] [Local Dock Goal Result] Cancelled!')

        elif status == GoalStatus.STATUS_ABORTED:
            self.node.get_logger().info(f'[{LOGGING_NAME}] [Local Dock Goal Result] Aborted!')
        else:
            self.node.get_logger().info(f'[{LOGGING_NAME}] [Local Dock Goal Result] Finished with unknown status: {status}')

        self._local_dock_goal_handle = None
        self._local_dock_running = False

    ##################################
    # Undock Action Client Functions #
    ##################################
    def send_undock_goal(self, dock_name):
        goal_msg = Undock.Goal()
        goal_msg.dock_name = dock_name

        if not self._undock_ac.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error(f'[{LOGGING_NAME}] autonomy/undock action server not available!')
            self.destroy_node()
            return

        self._send_goal_future = self._undock_ac.send_goal_async(goal_msg, feedback_callback=self.undock_feedback_cb)
        self._send_goal_future.add_done_callback(self.undock_goal_response_cb)

    def undock_goal_response_cb(self, future):
        self._undock_goal_handle = future.result()
        if not self._undock_goal_handle.accepted:
            self.node.get_logger().info(f'[{LOGGING_NAME}] Undock goal rejected')
            return

        self._undock_running = True
        self.node.get_logger().info(f'[{LOGGING_NAME}] Undock goal accepted')

        self._get_result_future = self._undock_goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.undock_get_result_cb)

    def undock_feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.node.get_logger().info(f'[{LOGGING_NAME}] Undocking has been running for: {feedback.elapsed_time:.2f}s', throttle_duration_sec=5)

    def undock_cancel_response_cb(self, future):
        cancel_response = future.result()
        self.get_logger().info(f'[{LOGGING_NAME}] Cancel response received: {cancel_response.return_code}')
        if len(cancel_response.goals_canceling) > 0:
            self.node.get_logger().info(f'[{LOGGING_NAME}] Cancelling of undock goal complete')
        else:
            self.node.get_logger().warning(f'[{LOGGING_NAME}] Undock goal failed to cancel')

    def undock_get_result_cb(self, future):
        result = future.result().result
        status = future.result().status

        self._docked = True

        # Compare the status integer against the constants defined in the message
        if status == GoalStatus.STATUS_SUCCEEDED:
            if result.success:
                self.node.get_logger().info(f'[{LOGGING_NAME}] [Undock Goal Result] Succeeded! Result: Robot undocked successfully!')
                self._docked = False
            else:
                self.node.get_logger().info(f'[{LOGGING_NAME}] [Undock Goal Result] Succeeded! Result: Robot failed to undock')
                self.node.destroy_node()
                rclpy.shutdown()

        elif status == GoalStatus.STATUS_CANCELED:
            self.node.get_logger().info(f'[{LOGGING_NAME}] [Undock Goal Result] Cancelled!')

        elif status == GoalStatus.STATUS_ABORTED:
            self.node.get_logger().info(f'[{LOGGING_NAME}] [Undock Goal Result] Aborted!')
            self._docked = False
        else:
            self.node.get_logger().info(f'[{LOGGING_NAME}] [Undock Goal Result] Finished with unknown status: {status}')

        self._undock_goal_handle = None
        self._undock_running = False

    def send_get_param_request(self, params_name_list: list[str]):
        self.req.names = params_name_list

        self.future = self._param_client.call_async(self.req)
        rclpy.spin_until_future_complete(self.node, self.future)
        return self.future.result()