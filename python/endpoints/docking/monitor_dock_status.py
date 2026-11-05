from clearpath_config.common.utils.yaml import read_yaml
from clearpath_dock_msgs.msg import DockStatus
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

ROBOT_CONFIG_PATH = '/etc/clearpath/robot.yaml'
LOGGING_NAME = 'MonitorDockStatus'
# The monitor publishes at 10 Hz; treat a longer silence as the monitor being unavailable.
STATUS_TIMEOUT_SEC = 1.0

STATES = {
    DockStatus.UNKNOWN: 'UNKNOWN',
    DockStatus.UNDOCKED: 'UNDOCKED',
    DockStatus.DOCKED: 'DOCKED',
}
CHARGING_STATES = {
    DockStatus.CHARGE_UNKNOWN: 'UNKNOWN',
    DockStatus.NOT_CHARGING: 'NOT_CHARGING',
    DockStatus.CHARGING: 'CHARGING',
    DockStatus.FULL: 'FULL',
}
EVIDENCE = {
    DockStatus.EVIDENCE_NONE: 'none',
    DockStatus.EVIDENCE_GEOMETRY: 'geometry',
    DockStatus.EVIDENCE_HARDWARE: 'hardware',
    DockStatus.EVIDENCE_BOTH: 'geometry+hardware',
}


class DockStatusMonitor(Node):

    def __init__(self, namespace: str = ''):
        super().__init__('dock_status_monitor')
        topic = f'/{namespace}/docking/status' if namespace else 'docking/status'

        # Transient-local delivers the latest status immediately on connection.
        qos = QoSProfile(depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_sub = self.create_subscription(DockStatus, topic, self.status_cb, qos)
        self.last_status = None
        self.last_key = None
        self.last_received = None
        self.stale_reported = False

        self.get_logger().info(f'[{LOGGING_NAME}] Monitoring {topic}')
        self.monitor_timer = self.create_timer(5.0, self.output_status)
        self.liveness_timer = self.create_timer(0.5, self.check_liveness)

    def status_cb(self, msg: DockStatus):
        self.last_status = msg
        self.last_received = self.get_clock().now()
        if self.stale_reported:
            self.get_logger().info(f'[{LOGGING_NAME}] Dock status stream resumed')
            self.stale_reported = False

        key = (msg.state, msg.charging_state, msg.conflict, msg.charger_fault)
        if key != self.last_key:
            self.last_key = key
            self.log_status(msg, changed=True)

    def check_liveness(self):
        if self.last_received is None or self.stale_reported:
            return
        age = (self.get_clock().now() - self.last_received).nanoseconds * 1e-9
        if age > STATUS_TIMEOUT_SEC:
            self.stale_reported = True
            self.get_logger().warn(
                f'[{LOGGING_NAME}] No dock status for {age:.1f}s; treat physical dock state as UNKNOWN')

    def output_status(self):
        if self.last_status is None:
            self.get_logger().info(f'[{LOGGING_NAME}] Waiting for dock status...')
        elif not self.stale_reported:
            self.log_status(self.last_status, changed=False)

    def log_status(self, msg: DockStatus, changed: bool):
        prefix = 'Dock status changed' if changed else 'Dock status'
        text = (f'[{LOGGING_NAME}] {prefix}: state = {STATES.get(msg.state, msg.state)}, '
                f'charging = {CHARGING_STATES.get(msg.charging_state, msg.charging_state)}, '
                f'evidence = {EVIDENCE.get(msg.evidence, msg.evidence)}, '
                f'template = {msg.dock_template or "n/a"}, '
                f'conflict = {msg.conflict}, charger_fault = {msg.charger_fault}, '
                f'reason = "{msg.reason}"')
        # rclpy requires a fixed severity per call site, so use separate calls.
        if msg.conflict or msg.charger_fault:
            self.get_logger().warn(text)
        else:
            self.get_logger().info(text)
        if msg.dock_target_tracked:
            self.get_logger().info(
                f'[{LOGGING_NAME}] Pose error from docked pose: x = {msg.pose_error.x:.3f} m, '
                f'y = {msg.pose_error.y:.3f} m, yaw = {msg.pose_error.theta:.3f} rad')

    def is_docked(self) -> bool:
        """Return True only for a fresh, confirmed DOCKED status."""
        return (self.last_status is not None and not self.stale_reported
                and self.last_status.state == DockStatus.DOCKED)


def main(args=None):
    rclpy.init(args=args)
    robot_config = read_yaml(ROBOT_CONFIG_PATH)
    namespace = robot_config['system']['ros2']['namespace']

    dock_status_monitor = DockStatusMonitor(namespace)
    try:
        rclpy.spin(dock_status_monitor)
    except KeyboardInterrupt:
        pass
    finally:
        dock_status_monitor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
