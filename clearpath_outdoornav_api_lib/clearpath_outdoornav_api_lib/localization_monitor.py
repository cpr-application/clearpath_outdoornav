#!/usr/bin/python3

from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from rclpy.node import Node
from tf_transformations import euler_from_quaternion

LOGGING_NAME = 'LocalizationMonitor'


class LocalizationMonitor(Node):

    def __init__(self, node: Node, namespace: str = ""):
        self.node = node
        odom_topic = f'/{namespace}/localization/odom' if namespace else 'localization/odom'
        fix_topic = f'/{namespace}/localization/fix' if namespace else 'localization/fix'

        self.odom_sub = self.node.create_subscription(
            Odometry, odom_topic, self.odom_cb, 10)

        self.fix_sub = self.node.create_subscription(
            NavSatFix, fix_topic, self.fix_cb, 10)

        self.odom_cb  # prevent unused variable warning
        self.fix_cb  # prevent unused variable warning
        self.last_odom_msg = Odometry()
        self.last_fix_msg = NavSatFix()

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