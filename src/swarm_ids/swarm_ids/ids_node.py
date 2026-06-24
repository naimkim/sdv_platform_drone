import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from swarm_interfaces.msg import SecurityAlert
from swarm_interfaces.msg import SignedPose
from swarm_interfaces.msg import SwarmHeartbeat

from swarm_security import secoc


# Node Config VARs
DEBUG = False
ALERT_LOG_COOLDOWN_SEC = 3.0

# Plausibility bounds for the search volume.
POS_XY_MIN = -2.0
POS_XY_MAX = 12.0
POS_Z_MIN = 0.0
POS_Z_MAX = 8.0
VEL_ABS_MAX = 1.5            # m/s, agent MAX_SPEED + margin

# A trusted member never jumps faster than physics allows between updates.
TELEPORT_MAX_JUMP = 2.0     # m between consecutive poses
MIN_POSE_INTERVAL_SEC = 0.02  # faster than this = flooding / spoof burst


class SwarmIDS(Node):
    """Intrusion detection for the swarm bus.

    Two defensive layers, mirroring vehicle CAN IDS practice:
      1. Cryptographic — verify the SecOC MAC and freshness. Catches outsiders
         who lack the group key and replays of captured frames.
      2. Plausibility — range, rate and teleport checks on the decoded state.
         Catches a compromised insider whose MAC is valid but whose claimed
         state is physically impossible.
    """

    def __init__(self):
        super().__init__('swarm_ids')

        self.declare_parameter('secret', 'sentinel-swarm-group-key')
        self.key = secoc.derive_key(self.get_parameter('secret').value)

        self.freshness = secoc.FreshnessVerifier()
        self.last_pose = {}
        self.last_pose_ns = {}
        self.last_alert_ns = {}

        self.alert_publisher = self.create_publisher(
            SecurityAlert, '/swarm/security_alert', 10)

        self.pose_subscription = self.create_subscription(
            SignedPose, '/swarm/pose', self.pose_callback, 10)
        self.heartbeat_subscription = self.create_subscription(
            SwarmHeartbeat, '/swarm/heartbeat', self.heartbeat_callback, 10)

        self.get_logger().info('Swarm IDS started')

    # -- pose path ------------------------------------------------------

    def pose_callback(self, msg):
        payload = secoc.pose_payload(
            msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz)

        if not secoc.verify_mac(
                self.key, msg.drone_id, msg.freshness, payload, msg.mac):
            self.raise_alert(
                msg.drone_id, 'MAC_INVALID', SecurityAlert.SEVERITY_ERROR,
                'Pose MAC verification failed (forged or wrong key)')
            return

        if not self.freshness.check('pose:' + msg.drone_id, msg.freshness):
            self.raise_alert(
                msg.drone_id, 'REPLAY', SecurityAlert.SEVERITY_ERROR,
                f'Stale freshness {msg.freshness} (replayed pose)')
            return

        self.check_pose_plausibility(msg)

    def check_pose_plausibility(self, msg):
        now_ns = self.get_clock().now().nanoseconds

        if not (POS_XY_MIN <= msg.x <= POS_XY_MAX and
                POS_XY_MIN <= msg.y <= POS_XY_MAX and
                POS_Z_MIN <= msg.z <= POS_Z_MAX):
            self.raise_alert(
                msg.drone_id, 'RANGE', SecurityAlert.SEVERITY_ERROR,
                f'Position out of bounds ({msg.x:.1f}, {msg.y:.1f}, '
                f'{msg.z:.1f})')

        if (abs(msg.vx) > VEL_ABS_MAX or abs(msg.vy) > VEL_ABS_MAX or
                abs(msg.vz) > VEL_ABS_MAX):
            self.raise_alert(
                msg.drone_id, 'RANGE', SecurityAlert.SEVERITY_ERROR,
                f'Velocity out of bounds ({msg.vx:.1f}, {msg.vy:.1f}, '
                f'{msg.vz:.1f})')

        last = self.last_pose.get(msg.drone_id)
        if last is not None:
            jump = math.dist((msg.x, msg.y, msg.z), last)
            if jump > TELEPORT_MAX_JUMP:
                self.raise_alert(
                    msg.drone_id, 'TELEPORT', SecurityAlert.SEVERITY_ERROR,
                    f'Position jump of {jump:.1f} m between updates')

        last_ns = self.last_pose_ns.get(msg.drone_id)
        if last_ns is not None:
            interval = (now_ns - last_ns) / 1_000_000_000.0
            if 0.0 <= interval < MIN_POSE_INTERVAL_SEC:
                self.raise_alert(
                    msg.drone_id, 'RATE', SecurityAlert.SEVERITY_WARN,
                    f'Pose flooding, interval {interval * 1000:.0f} ms')

        self.last_pose[msg.drone_id] = (msg.x, msg.y, msg.z)
        self.last_pose_ns[msg.drone_id] = now_ns

    # -- heartbeat path -------------------------------------------------

    def heartbeat_callback(self, msg):
        payload = secoc.heartbeat_payload(msg.stamp_ns)
        if not secoc.verify_mac(
                self.key, msg.drone_id, msg.freshness, payload, msg.mac):
            self.raise_alert(
                msg.drone_id, 'MAC_INVALID', SecurityAlert.SEVERITY_ERROR,
                'Heartbeat MAC verification failed')
            return
        if not self.freshness.check('hb:' + msg.drone_id, msg.freshness):
            self.raise_alert(
                msg.drone_id, 'REPLAY', SecurityAlert.SEVERITY_WARN,
                'Replayed heartbeat')

    # -- alerting -------------------------------------------------------

    def raise_alert(self, suspect_id, alert_type, severity, description):
        now_ns = self.get_clock().now().nanoseconds
        cooldown_key = (suspect_id, alert_type)
        last_ns = self.last_alert_ns.get(cooldown_key)
        if last_ns is not None:
            elapsed = (now_ns - last_ns) / 1_000_000_000.0
            if elapsed < ALERT_LOG_COOLDOWN_SEC:
                return
        self.last_alert_ns[cooldown_key] = now_ns

        msg = SecurityAlert()
        msg.source = 'swarm_ids'
        msg.suspect_id = suspect_id
        msg.alert_type = alert_type
        msg.severity = severity
        msg.description = description
        msg.stamp_ns = now_ns
        self.alert_publisher.publish(msg)

        self.get_logger().warn(
            f'[{alert_type}] {suspect_id}: {description}')


def main(args=None):
    rclpy.init(args=args)
    node = SwarmIDS()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
