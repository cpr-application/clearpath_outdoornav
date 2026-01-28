from clearpath_config.common.utils.yaml import read_yaml
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from tf_transformations import euler_from_quaternion

ROBOT_CONFIG_PATH = '/etc/clearpath/robot.yaml'
LOGGING_NAME = 'MonitorLocalization'


class LocalizationMonitor(Node):

    def __init__(self, namespace: str = ""):
        super().__init__('localization_monitor')

        self.odom_sub = self.create_subscription(
            Odometry, f'/{namespace}/localization/odom', self.odom_cb, 10)

        self.fix_sub = self.create_subscription(
            NavSatFix, f'/{namespace}/localization/fix', self.fix_cb, 10)

        self.odom_cb  # prevent unused variable warning
        self.fix_cb  # prevent unused variable warning
        self.last_odom_msg = Odometry()
        self.last_fix_msg = NavSatFix()

        # initialize monitor timer
        self.monitor_timer = self.create_timer(10, self.output_localization)

    def output_localization(self):
        pose = self.get_pose()
        (lat, lon) = self.get_fix()
        self.get_logger().info(f'[{LOGGING_NAME}] Localization (Odom): x = {pose.x:.3f}, y = {pose.y:.3f}, yaw = {pose.theta:.3f}')
        self.get_logger().info(f'[{LOGGING_NAME}] Localization (Fix): latitude = {lat:.7f}. longitude = {lon:.7f}')

    def odom_cb(self, msg):
        self.last_odom_msg = msg

    def fix_cb(self, msg):
        self.last_fix_msg = msg

    def get_pose(self) -> Pose2D:
        pose = Pose2D
        pose.x = self.last_odom_msg.pose.pose.position.x
        pose.y = self.last_odom_msg.pose.pose.position.y

        q = self.last_odom_msg.pose.pose.orientation
        q_list = [q.x, q.y, q.z, q.w]
        (_, _, yaw) = euler_from_quaternion(q_list)
        pose.theta = yaw
        return pose

    def get_fix(self) -> tuple:
        return (self.last_fix_msg.latitude, self.last_fix_msg.longitude)


def main(args=None):
    rclpy.init(args=args)
    robot_config = read_yaml(ROBOT_CONFIG_PATH)
    namespace = robot_config['system']['ros2']['namespace']

    localization_monitor = LocalizationMonitor(namespace)
    rclpy.spin(localization_monitor)


if __name__ == '__main__':
    main()
