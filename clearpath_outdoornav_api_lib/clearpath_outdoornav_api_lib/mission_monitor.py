#!/usr/bin/python3

from clearpath_mission_manager_msgs.msg import StorageState
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import String

LOGGING_NAME = 'MissionMonitor'


class MissionMonitor(Node):

    def __init__(self, node: Node, namespace: str = ""):
        self.node = node
        storage_state_sub_topic = f'/{namespace}/mission_manager/state' if namespace else 'mission_manager/state'
        current_goal_id_sub_topic = f'/{namespace}/navigation/current_goal_id' if namespace else 'navigation/current_goal_id'

        latching_qos = QoSProfile(depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.storage_state_sub = self.node.create_subscription(
            String, storage_state_sub_topic, self.storage_state_cb, qos_profile=latching_qos)

        self.current_goal_id_sub = self.node.create_subscription(
            String, current_goal_id_sub_topic, self.current_goal_id_cb, 10)

        self.storage_state_cb  # prevent unused variable warning
        self.current_goal_id_cb  # prevent unused variable warning

        self._last_storage_state_msg = None
        self._last_current_goal_id_msg = String()

        self._current_goal_id = ''
        self._last_completed_goal_id = ''
        self._next_goal_id = ''

    def storage_state_cb(self, msg):
        self.node.get_logger().info(f'[{LOGGING_NAME}] got storage state msg')
        self._last_storage_state_msg = msg

        for mission in msg.network_missions:
            if self._mission_running_id == mission.uuid:
                self._mission_running_waypoints = mission.waypoints
                break

    def current_goal_id_cb(self, msg):
        self._last_current_goal_id_msg = msg

        # get next goal ID as well for when we need to resume a mission
        self._current_goal_id = self._last_current_goal_id_msg.data

        if self._last_storage_state_msg is not None:
            for i in range(len(self._mission_running_waypoints)):
                if self._current_goal_id == self._mission_running_waypoints[i].uuid:
                    self._last_completed_goal_id = self._mission_running_waypoints[i - 1].uuid
                    self.next_goal_id = self._mission_running_waypoints[i + 1].uuid
                    break

        # self.node.get_logger().info(f'[{LOGGING_NAME}] current goal id: {self._current_goal_id}')

    def get_mission_running_id(self) -> String:
        return self._mission_running_id

    def get_current_goal_id(self) -> String:
        return self._current_goal_id

    def get_next_goal_id(self) -> String:
        return self._next_goal_id

    def get_last_completed_goal_id(self) -> String:
        return self._last_completed_goal_id

    def set_mission_running_id(self, mission_id):
        self._mission_running_id = mission_id