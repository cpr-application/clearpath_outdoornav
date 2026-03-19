#!/usr/bin/env python3
from clearpath_config.common.utils.yaml import read_yaml
from clearpath_motor_msgs.msg import LynxMotorProtection
from clearpath_outdoornav_api_lib.autonomy import Autonomy
from clearpath_outdoornav_api_lib.battery_monitor import BatteryMonitor
from clearpath_outdoornav_api_lib.disk_utils import DiskUtils
from clearpath_outdoornav_api_lib.localization_monitor import LocalizationMonitor
from clearpath_outdoornav_api_lib.mission_executor import MissionExecutor
from clearpath_outdoornav_api_lib.mission_monitor import MissionMonitor
from clearpath_outdoornav_api_lib.motors_monitor import MotorsMonitor
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

LOGGING_NAME = 'MissionWithMonitoring'

ROBOT_CONFIG_PATH = '/etc/clearpath/robot.yaml'

LOOP_TEST_MAP_ID = ''
LOOP_TEST_MISSION_ID = ''


class MissionWithMonitoring(Node):

    def __init__(self, namespace):
        super().__init__('run_mission', namespace=namespace)

        self._mission_started = False

        self._return_to_dock_conditions_passed = True
        self._prev_return_to_dock_conditions_passed = self._return_to_dock_conditions_passed

        self._pause_conditions_passed = True

        # initialize battery monitor
        self._battery_monitor = BatteryMonitor(self)
        self._battery_charge = self._battery_monitor.get_battery_percentage()

        # initialize motors monitor
        self._motors_monitor = MotorsMonitor(self)
        self._motor_states = self._motors_monitor.get_states()

        # initialize localization monitor
        self._localization_monitor = LocalizationMonitor(self)
        self._robot_fix = self._localization_monitor.get_fix()

        # initialize mission monitor
        self._mission_monitor = MissionMonitor(self)
        self._mission_monitor.set_mission_running_id(LOOP_TEST_MISSION_ID)
        self._currrent_goal_id = self._mission_monitor.get_current_goal_id()
        self._last_completed_goal_id = self._mission_monitor.get_last_completed_goal_id()
        self._next_goal_id = self._mission_monitor.get_next_goal_id()

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

        self._battery_charge = self._battery_monitor.get_battery_percentage()

        self._robot_fix = self._localization_monitor.get_fix()

        self._motor_states = self._motors_monitor.get_states()

        self._currrent_goal_id = self._mission_monitor.get_current_goal_id()
        self._last_completed_goal_id = self._mission_monitor.get_last_completed_goal_id()
        self._next_goal_id = self._mission_monitor.get_next_goal_id()

        self._mission_running = self._mission_executor._mission_running
        self._mission_completed = self._mission_executor._mission_completed
        self._mission_goal_handle = self._mission_executor._mission_goal_handle
        self._mission_goal_cancelled = self._mission_executor._mission_goal_cancelled

        if self._mission_completed:
            self.get_logger().info(f'[{LOGGING_NAME}] Mission completed!')
            self.destroy_node()
            rclpy.shutdown()
            return

        if not self._mission_running:
            if not self._mission_started:
                self.get_logger().info(f'[{LOGGING_NAME}] Starting mission')
                self._mission_executor._mission_running = True
                self._mission_executor.send_mission_goal(LOOP_TEST_MISSION_ID, LOOP_TEST_MAP_ID)
        else:
            self.get_logger().info(f"""[{LOGGING_NAME}]
--- Report ---
Current Goal ID: {self._currrent_goal_id}
Robot Location (lat/lon): ({self._robot_fix[0]}/{self._robot_fix[0]})
Battery Charge (%): {self._battery_charge*100}
Motor State: {self._motor_states}
Autonomy State: {self._autonomy_state}
Autonomy Paused: {self._autonomy_paused}
""", throttle_duration_sec=10.0)


def main(args=None):
    try:
        rclpy.init(args=args)
        robot_config = read_yaml(ROBOT_CONFIG_PATH)
        namespace = robot_config['system']['ros2']['namespace']

        mission = MissionWithMonitoring(namespace)

        rclpy.spin(mission)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
