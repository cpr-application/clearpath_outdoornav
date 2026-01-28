from clearpath_config.common.utils.yaml import read_yaml
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

ROBOT_CONFIG_PATH = '/etc/clearpath/robot.yaml'
LOGGING_NAME = 'ResumeAutonomy'


class ResumeAutonomy(Node):

    def __init__(self, namespace: str):
        super().__init__('resume_autonomy_client')
        self._autonomy_resume_client = self.create_client(SetBool, f'/{namespace}/autonomy/resume')

        while not self._autonomy_resume_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info(f'[{LOGGING_NAME}] /{namespace}/autonomy/resume service not available, waiting again...')

    def resume(self):
        req = SetBool.Request()
        req.data = True
        self.future = self._autonomy_resume_client.call_async(req)
        self.future.add_done_callback(self.autonomy_resume_srv_response_cb)

    def autonomy_resume_srv_response_cb(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'[{LOGGING_NAME}] Autonomy resumed!')
            else:
                self.get_logger().error(f'[{LOGGING_NAME}] Failed to resume autonomy')
        except Exception as e:
            self.get_logger().error(f'[{LOGGING_NAME}] Service call failed: {e}')

        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    robot_config = read_yaml(ROBOT_CONFIG_PATH)
    namespace = robot_config['system']['ros2']['namespace']

    resume_autonomy_client = ResumeAutonomy(namespace)
    resume_autonomy_client.resume()

    rclpy.spin(resume_autonomy_client)


if __name__ == '__main__':
    main()
