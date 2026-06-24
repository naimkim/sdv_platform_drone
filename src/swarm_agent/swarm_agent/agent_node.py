import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from swarm_interfaces.msg import SignedPose
from swarm_interfaces.msg import SwarmHeartbeat
from swarm_interfaces.msg import SwarmStatus

from swarm_security import secoc

from swarm_coordination import allocation, collision, coverage


# Node Config VARs
DEBUG = False

POSE_RATE_HZ = 10.0
HEARTBEAT_RATE_HZ = 1.0
MAX_SPEED = 0.6              # m/s
ARRIVAL_RADIUS = 0.3        # m, considered "on station"
SEPARATION_RADIUS = 1.0     # m, start avoiding peers
SEPARATION_GAIN = 0.5
AREA_SIZE = 10.0            # m, square search area side
COVERAGE_SPACING = 2.0     # m, lawnmower pass spacing within a sector


class DroneAgent(Node):
    """A single swarm member.

    Broadcasts SecOC-authenticated pose/heartbeat, derives its own search sector
    deterministically from the consensus trusted set (Phase 4 coordination), and
    flies a lawnmower coverage path through it while avoiding peers. When the
    consensus guard quarantines a member, the trusted set shrinks and every
    honest agent re-partitions and re-plans coverage with no extra round.
    """

    def __init__(self):
        super().__init__('swarm_agent')

        self.declare_parameter('drone_id', 'drone_1')
        self.declare_parameter('secret', 'sentinel-swarm-group-key')
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        # A compromised insider still holds the group key, so its MACs verify.
        # Its lies are caught by the IDS plausibility checks instead.
        self.declare_parameter('malicious', False)

        self.drone_id = self.get_parameter('drone_id').value
        self.key = secoc.derive_key(self.get_parameter('secret').value)
        self.malicious = bool(self.get_parameter('malicious').value)

        self.x = float(self.get_parameter('start_x').value)
        self.y = float(self.get_parameter('start_y').value)
        self.z = 2.0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        self.freshness = self.get_clock().now().nanoseconds // 1_000_000
        self.trusted = [self.drone_id]
        self.peer_pose = {}

        # Coverage state: regenerated when the trusted membership changes.
        self.coverage_wps = []
        self.cov_index = 0
        self.cov_signature = None

        self.pose_publisher = self.create_publisher(
            SignedPose, '/swarm/pose', 10)
        self.heartbeat_publisher = self.create_publisher(
            SwarmHeartbeat, '/swarm/heartbeat', 10)

        self.pose_subscription = self.create_subscription(
            SignedPose, '/swarm/pose', self.pose_callback, 10)
        self.status_subscription = self.create_subscription(
            SwarmStatus, '/swarm/status', self.status_callback, 10)

        self.pose_timer = self.create_timer(
            1.0 / POSE_RATE_HZ, self.pose_tick)
        self.heartbeat_timer = self.create_timer(
            1.0 / HEARTBEAT_RATE_HZ, self.heartbeat_tick)

        self.get_logger().info(
            f'Agent {self.drone_id} started '
            f'(malicious={self.malicious}) at ({self.x:.1f}, {self.y:.1f})')

    # -- subscriptions --------------------------------------------------

    def pose_callback(self, msg):
        if msg.drone_id == self.drone_id:
            return
        self.peer_pose[msg.drone_id] = (msg.x, msg.y)

    def status_callback(self, msg):
        # Only honest, trusted members participate in area allocation.
        if msg.trusted_drones:
            self.trusted = list(msg.trusted_drones)

    # -- control loop ---------------------------------------------------

    def pose_tick(self):
        self.update_coverage()
        dt = 1.0 / POSE_RATE_HZ

        target = self.coverage_wps[self.cov_index]
        dist = math.hypot(target[0] - self.x, target[1] - self.y)
        if dist <= ARRIVAL_RADIUS:
            # Loop through the sector's coverage path.
            self.cov_index = (self.cov_index + 1) % len(self.coverage_wps)
            target = self.coverage_wps[self.cov_index]
            dist = math.hypot(target[0] - self.x, target[1] - self.y)

        if dist > 1e-6:
            ux, uy = (target[0] - self.x) / dist, (target[1] - self.y) / dist
            desired = (ux * MAX_SPEED, uy * MAX_SPEED)
        else:
            desired = (0.0, 0.0)

        # Inter-drone collision avoidance (Phase 4 coordination library).
        neighbors = list(self.peer_pose.values())
        vx, vy = collision.avoid(
            desired, (self.x, self.y), neighbors,
            SEPARATION_RADIUS, SEPARATION_GAIN, MAX_SPEED)

        self.x += vx * dt
        self.y += vy * dt
        self.vx, self.vy = vx, vy

        self.publish_pose()

    def update_coverage(self):
        """Re-derive the sector and coverage path when membership changes."""
        members = self.trusted if self.drone_id in self.trusted \
            else self.trusted + [self.drone_id]
        signature = tuple(sorted(set(members)))
        if signature == self.cov_signature and self.coverage_wps:
            return

        bounds = allocation.vertical_strip_bounds(
            self.drone_id, members, AREA_SIZE)
        self.coverage_wps = coverage.lawnmower(bounds, COVERAGE_SPACING)
        self.cov_index = 0
        self.cov_signature = signature

    # -- authenticated publishing --------------------------------------

    def next_freshness(self):
        self.freshness += 1
        return self.freshness

    def publish_pose(self):
        msg = SignedPose()
        msg.drone_id = self.drone_id
        msg.x, msg.y, msg.z = self.x, self.y, self.z
        msg.vx, msg.vy, msg.vz = self.vx, self.vy, self.vz
        msg.freshness = self.next_freshness()
        msg.stamp_ns = self.get_clock().now().nanoseconds

        payload = secoc.pose_payload(
            msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz)
        msg.mac = secoc.compute_mac(
            self.key, self.drone_id, msg.freshness, payload)

        self.pose_publisher.publish(msg)

        if DEBUG:
            self.get_logger().info(
                f'{self.drone_id} pose ({self.x:.2f}, {self.y:.2f})')

    def heartbeat_tick(self):
        msg = SwarmHeartbeat()
        msg.drone_id = self.drone_id
        msg.freshness = self.next_freshness()
        msg.stamp_ns = self.get_clock().now().nanoseconds

        payload = secoc.heartbeat_payload(msg.stamp_ns)
        msg.mac = secoc.compute_mac(
            self.key, self.drone_id, msg.freshness, payload)

        self.heartbeat_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DroneAgent()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
