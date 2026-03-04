from action_msgs.msg import GoalStatus  # Import for reference to status constants
from clearpath_config.common.utils.yaml import read_yaml
from clearpath_task_msgs.action import ExecuteTask
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

ROBOT_CONFIG_PATH = '/etc/clearpath/robot.yaml'
LOGGING_NAME = 'ExecuteTask'

# TODO: Enter your task uuid
TASK_ID: str = ''


class ExecuteTaskActionClient(Node):

    def __init__(self, namespace: str):
        super().__init__('execute_task')
        self._namespace = namespace
        self._action_client = ActionClient(self, ExecuteTask, f'/{self._namespace}/execute_task')
        self._task_goal_handle = None
        self._task_goal_cancelled = False
        self._task_running = False
        self._task_completed = False

    ###################################
    # Task Action Client Functions #
    ###################################
    def send_goal(self, task_id):
        goal_msg = ExecuteTask.Goal()

        # populate goal message
        goal_msg.task_id = task_id

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'[{LOGGING_NAME}] /{self._namespace}/execute_task action server not available!')
            self.destroy_node()
            rclpy.shutdown()
            return

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.task_feedback_cb)
        self._send_goal_future.add_done_callback(self.task_goal_response_cb)

    def task_goal_response_cb(self, future):
        self._task_goal_handle = future.result()
        if not self._task_goal_handle.accepted:
            self.get_logger().info(f'[{LOGGING_NAME}] Task Goal rejected')
            self._task_goal_handle = None
            self.destroy_node()
            rclpy.shutdown()
            return

        self._task_running = True
        self._task_completed = False
        self._task_goal_cancelled = False
        self.get_logger().info(f'[{LOGGING_NAME}] Task Goal accepted')

        self._get_result_future = self._task_goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.task_get_result_cb)

    def task_feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'[{LOGGING_NAME}] Task has been running for: {feedback.time_elapsed:.2f}s', throttle_duration_sec=60)

    def task_cancel_response_cb(self, future):
        cancel_response = future.result()
        self.get_logger().info(f'[{LOGGING_NAME}] Cancel response received: {cancel_response.return_code}')
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info(f'[{LOGGING_NAME}] Canceling of task goal complete')
        else:
            self.get_logger().warning(f'[{LOGGING_NAME}] Task Goal failed to cancel')

    def task_get_result_cb(self, future):
        result = future.result().result
        status = future.result().status

        # Compare the status integer against the constants defined in the message
        if status == GoalStatus.STATUS_SUCCEEDED:
            if result.success:
                self.get_logger().info(f'[{LOGGING_NAME}] [Task Goal Result] Succeeded! Result: Robot completed task!')
                self._goto_completed = True
            else:
                self.get_logger().info(f'[{LOGGING_NAME}] [Task Goal Result] Succeeded! Result: Robot failed to complete task')

        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info(f'[{LOGGING_NAME}] [Task Goal Result] Cancelled!')
            self._task_goal_cancelled = True

        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().info(f'[{LOGGING_NAME}] [Task Goal Result] Aborted!')
        else:
            self.get_logger().info(f'[{LOGGING_NAME}] [Task Goal Result] Finished with unknown status: {status}')

        self._task_goal_handle = None
        self._task_running = False

        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    robot_config = read_yaml(ROBOT_CONFIG_PATH)
    namespace = robot_config['system']['ros2']['namespace']

    execute_task = ExecuteTaskActionClient(namespace)
    execute_task.send_goal(TASK_ID)
    rclpy.spin(execute_task)


if __name__ == '__main__':
    main()
