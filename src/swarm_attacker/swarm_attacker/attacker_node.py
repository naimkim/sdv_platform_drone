import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from swarm_interfaces.msg import SignedPose

from swarm_security import secoc


# Node Config VARs
DEBUG = False
ATTACK_RATE_HZ = 5.0

# Teleport waypoints for the compromised-insider scenario. Both are inside the
# arena (so RANGE stays clean) but far enough apart to trip TELEPORT.
TELEPORT_A = (1.0, 5.0, 2.0)
TELEPORT_B = (9.0, 5.0, 2.0)


class SwarmAttacker(Node):
    """Adversary traffic generator for the swarm bus.

    Three modes exercise both defensive layers:

      outsider_spoof   - impersonates a victim id without the group key.
                         The MAC is forged, so the IDS rejects it (MAC_INVALID).
      replay           - captures a genuine signed pose and rebroadcasts it.
                         The MAC is valid but the freshness is stale (REPLAY).
      insider_teleport - a compromised member that *does* hold the group key.
                         MACs verify, but the claimed motion is physically
                         impossible, so plausibility checks fire (TELEPORT).
    """

    def __init__(self):
        super().__init__('swarm_attacker')

        self.declare_parameter('attack_type', 'outsider_spoof')
        self.declare_parameter('target_id', 'drone_2')
        self.declare_parameter('secret', 'sentinel-swarm-group-key')

        self.attack_type = self.get_parameter('attack_type').value
        self.target_id = self.get_parameter('target_id').value
        group_secret = self.get_parameter('secret').value

        # Outsider only has a wrong key; insider/replay use the real group key.
        self.group_key = secoc.derive_key(group_secret)
        self.forged_key = secoc.derive_key(group_secret + ':forged')

        self.publisher = self.create_publisher(SignedPose, '/swarm/pose', 10)
        self.freshness = 10_000_000
        self.teleport_toggle = False
        self.captured = None

        if self.attack_type == 'replay':
            self.subscription = self.create_subscription(
                SignedPose, '/swarm/pose', self.capture_callback, 10)

        self.timer = self.create_timer(1.0 / ATTACK_RATE_HZ, self.attack_tick)

        self.get_logger().warn(
            f'Attacker active: mode={self.attack_type}, '
            f'target={self.target_id}')

    def capture_callback(self, msg):
        # Capture one genuine pose from the victim to replay later.
        if msg.drone_id == self.target_id and self.captured is None:
            self.captured = msg
            self.get_logger().warn(
                f'Captured genuine pose from {self.target_id} '
                f'(freshness={msg.freshness}) for replay')

    def attack_tick(self):
        if self.attack_type == 'outsider_spoof':
            self.send_outsider_spoof()
        elif self.attack_type == 'replay':
            self.send_replay()
        elif self.attack_type == 'insider_teleport':
            self.send_insider_teleport()
        else:
            self.get_logger().error(
                f'Unknown attack_type: {self.attack_type}')

    def send_outsider_spoof(self):
        # Plausible-looking pose, but signed with the wrong key.
        self.freshness += 1
        msg = self.build_pose(
            self.target_id, 5.0, 5.0, 2.0, self.freshness, self.forged_key)
        self.publisher.publish(msg)

    def send_replay(self):
        if self.captured is not None:
            # Rebroadcast verbatim: valid MAC, stale freshness.
            self.publisher.publish(self.captured)

    def send_insider_teleport(self):
        self.teleport_toggle = not self.teleport_toggle
        x, y, z = TELEPORT_A if self.teleport_toggle else TELEPORT_B
        self.freshness += 1
        # Valid MAC (insider holds the group key) but impossible jump.
        msg = self.build_pose(
            self.target_id, x, y, z, self.freshness, self.group_key)
        self.publisher.publish(msg)

    def build_pose(self, drone_id, x, y, z, freshness, key):
        msg = SignedPose()
        msg.drone_id = drone_id
        msg.x, msg.y, msg.z = x, y, z
        msg.vx, msg.vy, msg.vz = 0.0, 0.0, 0.0
        msg.freshness = freshness
        msg.stamp_ns = self.get_clock().now().nanoseconds
        payload = secoc.pose_payload(x, y, z, 0.0, 0.0, 0.0)
        msg.mac = secoc.compute_mac(key, drone_id, freshness, payload)
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = SwarmAttacker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
