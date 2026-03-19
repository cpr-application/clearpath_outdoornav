#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, TransformStamped
from nav_msgs.msg import Odometry
from clearpath_localization_msgs.msg import XvnStatus
from sensor_msgs.msg import NavSatFix
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import math
import time
from tf2_msgs.msg import TFMessage  # Add this import at the top
from clearpath_config.common.utils.yaml import read_yaml

LOGGING_NAME = 'LocalizationSim'
ROBOT_CONFIG_PATH = '/etc/clearpath/robot.yaml'

class LocalizationSim(Node):
    def __init__(self, namespace):
        super().__init__('localization_sim', namespace=namespace)

        # Create QoS profile for better reliability
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE)

        # Create latched QoS profile for GPS
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # Initialize subscribers and publishers
        self.tf_pub = self.create_publisher(
            TFMessage,
            'tf',
            qos_profile)

        self.cmd_vel_sub = self.create_subscription(
            TwistStamped,
            'ui_teleop/cmd_vel',
            self.cmd_vel_callback,
            qos_profile)

        self.cmd_vel_sub_autonomy = self.create_subscription(
            TwistStamped,
            'cmd_vel',
            self.cmd_vel_callback,
            qos_profile)

        self.odom_pub = self.create_publisher(
            Odometry,
            'localization/odom',
            qos_profile)

        self.odom_pub2 = self.create_publisher(
            Odometry,
            'localization/odom_throttle',
            qos_profile)

        self.platform_pub = self.create_publisher(
            Odometry,
            'platform/odom',
            qos_profile)

        # GPS/NavSatFix publisher (latched)
        self.navsatfix_pub = self.create_publisher(
            NavSatFix,
            'localization/datum',
            latched_qos)

        self.xvn_status_pub = self.create_publisher(
            XvnStatus,
            'localization/xvn_status',
            qos_profile)

        self.current_twist = None
        # Initialize odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = None
        self.frame_id = 'odom'
        self.child_frame_id = 'base_link'
        self.publish_rate = 50

        # Constant GPS coordinates
        # Clearpath Testing Area
        self.latitude = 43.500644
        self.longitude = -80.547214

        # Sunlife parking lot
        #self.latitude = 43.5002980
        #self.longitude = -80.5487636

        # Shipping Entrance
        #self.latitude = 43.5007041
        #self.longitude = -80.5464208

        self.altitude = 76.0  # meters above sea level

        # Publish initial GPS fix (latched)
        self.publish_initial_gps_fix()

        self.create_timer(1.0/self.publish_rate, self.timer_callback)
        self.create_timer(0.2, self.timer_slow_cb)
        self.get_logger().info('Localization Sim Node Started')

    def publish_initial_gps_fix(self):
        """Publish a single latched GPS fix message with constant coordinates"""
        navsatfix_msg = NavSatFix()
        navsatfix_msg.header.stamp = self.get_clock().now().to_msg()
        navsatfix_msg.header.frame_id = 'base_link'

        # Set GPS coordinates
        navsatfix_msg.latitude = self.latitude
        navsatfix_msg.longitude = self.longitude
        navsatfix_msg.altitude = self.altitude

        # Set status (assume GPS fix is available)
        navsatfix_msg.status.status = 0  # STATUS_FIX
        navsatfix_msg.status.service = 1  # SERVICE_GPS

        # Set covariance (diagonal elements for lat, lon, alt uncertainty)
        # Set to reasonable GPS accuracy values (in meters squared)
        navsatfix_msg.position_covariance[0] = 2.0  # latitude variance
        navsatfix_msg.position_covariance[4] = 2.0  # longitude variance
        navsatfix_msg.position_covariance[8] = 4.0  # altitude variance
        navsatfix_msg.position_covariance_type = 2  # COVARIANCE_TYPE_DIAGONAL_KNOWN

        # Publish the latched message
        self.navsatfix_pub.publish(navsatfix_msg)
        self.get_logger().info(f'Published GPS fix: lat={self.latitude}, lon={self.longitude}, alt={self.altitude}')

    def publish_transform(self, timestamp):
        # Create odom->base_link transform
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = self.frame_id
        t.child_frame_id = self.child_frame_id

        # Set translation
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        # Set rotation
        t.transform.rotation.z = math.sin(self.theta/2.0)
        t.transform.rotation.w = math.cos(self.theta/2.0)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0

        # Create map->odom identity transform
        t_map_odom = TransformStamped()
        t_map_odom.header.stamp = timestamp
        t_map_odom.header.frame_id = 'map'
        t_map_odom.child_frame_id = 'odom'

        # Identity transform (no translation or rotation)
        t_map_odom.transform.translation.x = 0.0
        t_map_odom.transform.translation.y = 0.0
        t_map_odom.transform.translation.z = 0.0
        t_map_odom.transform.rotation.x = 0.0
        t_map_odom.transform.rotation.y = 0.0
        t_map_odom.transform.rotation.z = 0.0
        t_map_odom.transform.rotation.w = 1.0

        # Create TFMessage and publish both transforms
        tf_msg = TFMessage()
        tf_msg.transforms = [t_map_odom, t]
        self.tf_pub.publish(tf_msg)

    def timer_callback(self):
        # Create odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.frame_id
        odom_msg.child_frame_id = self.child_frame_id

        # Set position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Set orientation
        odom_msg.pose.pose.orientation.z = math.sin(self.theta/2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta/2.0)

        # Set current velocities
        if self.current_twist:
            odom_msg.twist.twist = self.current_twist

        # Publish
        self.odom_pub.publish(odom_msg)
        self.platform_pub.publish(odom_msg)
        timestamp = self.get_clock().now().to_msg()
        self.publish_transform(timestamp)

    def timer_slow_cb(self):
        # Create odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.frame_id
        odom_msg.child_frame_id = self.child_frame_id

        # Set position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Set orientation
        odom_msg.pose.pose.orientation.z = math.sin(self.theta/2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta/2.0)

        self.odom_pub2.publish(odom_msg)

        # Set current velocities
        if self.current_twist:
            odom_msg.twist.twist = self.current_twist

        # Set the dummy "all good" XVN status state for sim
        xvn_status_msg = XvnStatus()
        xvn_status_msg.gnss1_status = XvnStatus.GNSS_RTK_FIX
        xvn_status_msg.gnss2_status = XvnStatus.GNSS_RTK_FIX
        xvn_status_msg.fusion_status = XvnStatus.VI_GNSS
        xvn_status_msg.wheelspeed_status = XvnStatus.WS_CONVERGED
        xvn_status_msg.imu_status = XvnStatus.IMU_CONVERGED
        xvn_status_msg.rtk_status = XvnStatus.RTK_CONNECTED

        # Publish
        self.odom_pub.publish(odom_msg)
        self.platform_pub.publish(odom_msg)
        self.odom_pub2.publish(odom_msg)
        self.xvn_status_pub.publish(xvn_status_msg)

    def cmd_vel_callback(self, msg: TwistStamped):
        current_time = time.time() #self.get_clock().now() #msg.header.stamp

        # First message handling
        if self.last_time is None:
            self.last_time = current_time
            return
        if current_time - self.last_time > 1:
            self.last_time = current_time
            return
        self.current_twist = msg.twist 
        # Calculate time difference
        dt = current_time - self.last_time
        #dt = (current_time.sec - self.last_time.sec) + \
        #     (current_time.nanosec - self.last_time.nanosec) * 1e-9

        # Extract velocities
        v_x = msg.twist.linear.x
        v_y = msg.twist.linear.y
        omega = msg.twist.angular.z

        # Integrate velocities to get new pose
        if abs(omega) < 0.0001:  # Going straight
            self.x += v_x * math.cos(self.theta) * dt
            self.y += v_x * math.sin(self.theta) * dt
        else:  # Following arc
            self.x += (-v_x/omega) * math.sin(self.theta) + \
                     (v_x/omega) * math.sin(self.theta + omega * dt)
            self.y += (v_x/omega) * math.cos(self.theta) - \
                     (v_x/omega) * math.cos(self.theta + omega * dt)

        self.theta += omega * dt

        # Create and publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg() #current_time
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Set position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Set orientation (basic quaternion from yaw)
        odom_msg.pose.pose.orientation.z = math.sin(self.theta/2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta/2.0)

        # Set velocities
        odom_msg.twist.twist = msg.twist

        # Publish
        #self.odom_pub.publish(odom_msg)
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    robot_config = read_yaml(ROBOT_CONFIG_PATH)
    namespace = robot_config['system']['ros2']['namespace']
    node = LocalizationSim(namespace)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
