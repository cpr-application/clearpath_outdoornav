#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

LOGGING_NAME = 'Logger'


class Logger(Node):
    def __init__(self, node: Node, namespace: str = ""):
        self.node = node
        start_log_srv = f'/{namespace}/log_manager/start_recording' if namespace else 'log_manager/start_recording'
        stop_log_srv = f'/{namespace}/log_manager/stop_recording' if namespace else 'log_manager/stop_recording'

        self._start_logger_srv_client = self.node.create_client(Trigger, start_log_srv)
        while not self._start_logger_srv_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'[{LOGGING_NAME}] service not available, waiting again...')

        self._stop_logger_srv_client = self.node.create_client(Trigger, stop_log_srv)
        while not self._stop_logger_srv_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'[{LOGGING_NAME}] service not available, waiting again...')

        self.trigger_req = Trigger.Request()

    def start_recording(self):
        self.future = self._start_logger_srv_client.call_async(self.trigger_req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def stop_recording(self):
        self.future = self._stop_logger_srv_client.call_async(self.trigger_req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
