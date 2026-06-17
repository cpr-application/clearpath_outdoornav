#!/usr/bin/python3

from clearpath_control_selection_msgs.msg import ControlState
from clearpath_navigation_msgs.msg import AutonomyStatus
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger

LOGGING_NAME = 'Autonomy'


class Autonomy(Node):

    def __init__(self, node: Node, namespace: str = ""):
        self.node = node
        autonomy_status_topic = f'/{namespace}/autonomy/status' if namespace else 'autonomy/status'
        pause_srv = f'/{namespace}/autonomy/pause' if namespace else 'autonomy/pause'
        resume_srv = f'/{namespace}/autonomy/resume' if namespace else 'autonomy/resume'
        stop_srv = f'/{namespace}/autonomy/stop' if namespace else 'autonomy/stop'
        control_state_topic = f'/{namespace}/control_selection/control_state' if namespace else 'control_selection/control_state'

        self.autonomy_status_sub = self.node.create_subscription(
            AutonomyStatus, autonomy_status_topic, self.autonomy_status_cb, 10)
        self.autonomy_status_cb  # prevent unused variable warning
        self._current_autonomy_state = AutonomyStatus.IDLE

        self._autonomy_pause_client = self.node.create_client(SetBool, pause_srv)
        self._autonomy_resume_client = self.node.create_client(SetBool, resume_srv)
        self._autonomy_stop_client = self.node.create_client(Trigger, stop_srv)

        self.control_state_sub = self.node.create_subscription(
            ControlState, control_state_topic, self.control_state_cb, 10)
        self.control_state_cb  # prevent unused variable warning

        self._currently_paused = False

    def autonomy_status_cb(self, msg: AutonomyStatus):
        self._current_autonomy_state = msg.state
        self._currently_paused = msg.paused

    def control_state_cb(self, msg: ControlState):
        self._currently_paused = msg.autonomy.paused

    def get_state(self) -> int:
        return self._current_autonomy_state

    def is_paused(self) -> bool:
        return self._currently_paused

    def pause(self):
        req = SetBool.Request()
        req.data = True
        self.future = self._autonomy_pause_client.call_async(req)
        self.future.add_done_callback(self.autonomy_pause_srv_response_cb)

    def resume(self):
        req = SetBool.Request()
        req.data = True
        self.future = self._autonomy_resume_client.call_async(req)
        self.future.add_done_callback(self.autonomy_resume_srv_response_cb)

    def stop(self):
        req = Trigger.Request()
        self.future = self._autonomy_stop_client.call_async(req)
        self.future.add_done_callback(self.autonomy_stop_srv_response_cb)

    def autonomy_pause_srv_response_cb(self, future):
        try:
            response = future.result()
            if response.success:
                self.node.get_logger().info(f'[{LOGGING_NAME}] Autonomy paused!')
            else:
                self.node.get_logger().error(f'[{LOGGING_NAME}] Failed to pause autonomy')
        except Exception as e:
            self.node.get_logger().error(f'[{LOGGING_NAME}] Service call failed: {e}')

    def autonomy_resume_srv_response_cb(self, future):
        try:
            response = future.result()
            if response.success:
                self.node.get_logger().info(f'[{LOGGING_NAME}] Autonomy resumed!')
            else:
                self.node.get_logger().error(f'[{LOGGING_NAME}] Failed to resume autonomy')
        except Exception as e:
            self.node.get_logger().error(f'[{LOGGING_NAME}] Service call failed: {e}')

    def autonomy_stop_srv_response_cb(self, future):
        try:
            response = future.result()
            if response.success:
                self.node.get_logger().info(f'[{LOGGING_NAME}] Autonomy stopped!')
            else:
                self.node.get_logger().error(f'[{LOGGING_NAME}] Failed to stop autonomy')
        except Exception as e:
            self.node.get_logger().error(f'[{LOGGING_NAME}] Service call failed: {e}')

