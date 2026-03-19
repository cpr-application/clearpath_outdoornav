#!/usr/bin/env python3
from clearpath_config.common.utils.yaml import read_yaml
from clearpath_outdoornav_api_lib.autonomy import Autonomy
from clearpath_outdoornav_api_lib.mission_executor import MissionExecutor
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

LOGGING_NAME = 'MissionLooped'

ROBOT_CONFIG_PATH = '/etc/clearpath/robot.yaml'

LOOP_TEST_MAP_ID = ''
LOOP_TEST_MISSION_ID = ''


class MissionLooped(Node):

    def __init__(self, namespace):
        super().__init__('mission_looped', namespace=namespace)

        self._mission_running = False
        self._pause_conditions_passed = True

        # initialize autonomy monitor
        self._autonomy = Autonomy(self)
        self._autonomy_state = self._autonomy.get_state()
        self._autonomy_paused = self._autonomy.is_paused()

        # initialize mission executor
        self._mission_executor = MissionExecutor(self)

        # initialize monitor timer
        self._monitor_timer = self.create_timer(1, self.update_monitors)

    def update_monitors(self):

        self._autonomy_state = self._autonomy.get_state()
        self._autonomy_paused = self._autonomy.is_paused()

        self._mission_running = self._mission_executor._mission_running
        self._mission_goal_handle = self._mission_executor._mission_goal_handle
        self._mission_goal_cancelled = self._mission_executor._mission_goal_cancelled

        if not self._mission_running:
            self.get_logger().info(f'[{LOGGING_NAME}] Running mission')
            self._mission_running = True
            self._mission_executor.send_mission_goal(LOOP_TEST_MISSION_ID, LOOP_TEST_MAP_ID)


def main(args=None):
    try:
        rclpy.init(args=args)
        robot_config = read_yaml(ROBOT_CONFIG_PATH)
        namespace = robot_config['system']['ros2']['namespace']

        mission_loop = MissionLooped(namespace)

        rclpy.spin(mission_loop)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
