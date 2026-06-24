import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from swarm_interfaces.msg import ConsensusVote
from swarm_interfaces.msg import SecurityAlert
from swarm_interfaces.msg import SwarmHeartbeat
from swarm_interfaces.msg import SwarmStatus


# Node Config VARs
DEBUG = False
STATUS_PUBLISH_RATE_HZ = 1.0
MEMBER_TIMEOUT_SEC = 5.0


class ConsensusGuard(Node):
    """Byzantine-resilient quarantine consensus.

    Aggregates accusations (IDS alerts + peer votes) and quarantines a suspect
    only when the evidence clears two thresholds:

      * quorum     - at least N *distinct* accusers, so a single lying node
                     cannot evict an honest peer on its own.
      * min_alerts - sustained evidence, so one transient false positive does
                     not trip a quarantine (hysteresis).

    The resulting trusted set is broadcast as SwarmStatus; honest agents use it
    to re-divide the search area without quarantined members.
    """

    def __init__(self):
        super().__init__('swarm_consensus')

        self.declare_parameter('quorum', 1)
        self.declare_parameter('min_alerts', 3)
        self.quorum = int(self.get_parameter('quorum').value)
        self.min_alerts = int(self.get_parameter('min_alerts').value)

        self.members_last_ns = {}          # drone_id -> last heartbeat ns
        self.accusers = {}                 # suspect -> set(accuser ids)
        self.alert_count = {}              # suspect -> total accusation count
        self.quarantined = set()

        self.status_publisher = self.create_publisher(
            SwarmStatus, '/swarm/status', 10)

        self.alert_subscription = self.create_subscription(
            SecurityAlert, '/swarm/security_alert', self.alert_callback, 10)
        self.vote_subscription = self.create_subscription(
            ConsensusVote, '/swarm/vote', self.vote_callback, 10)
        self.heartbeat_subscription = self.create_subscription(
            SwarmHeartbeat, '/swarm/heartbeat', self.heartbeat_callback, 10)

        self.status_timer = self.create_timer(
            1.0 / STATUS_PUBLISH_RATE_HZ, self.publish_status)

        self.get_logger().info(
            f'Consensus guard started (quorum={self.quorum}, '
            f'min_alerts={self.min_alerts})')

    # -- evidence intake ------------------------------------------------

    def heartbeat_callback(self, msg):
        self.members_last_ns[msg.drone_id] = \
            self.get_clock().now().nanoseconds

    def alert_callback(self, msg):
        if msg.severity < SecurityAlert.SEVERITY_ERROR:
            return
        self.record_accusation(msg.suspect_id, msg.source)

    def vote_callback(self, msg):
        if msg.malicious:
            self.record_accusation(msg.suspect_id, msg.voter_id)

    def record_accusation(self, suspect_id, accuser_id):
        if not suspect_id:
            return
        self.accusers.setdefault(suspect_id, set()).add(accuser_id)
        self.alert_count[suspect_id] = self.alert_count.get(suspect_id, 0) + 1
        self.evaluate(suspect_id)

    def evaluate(self, suspect_id):
        if suspect_id in self.quarantined:
            return
        distinct = len(self.accusers.get(suspect_id, ()))
        count = self.alert_count.get(suspect_id, 0)
        if distinct >= self.quorum and count >= self.min_alerts:
            self.quarantined.add(suspect_id)
            self.get_logger().error(
                f'QUARANTINE {suspect_id}: {distinct} accuser(s), '
                f'{count} alert(s) -> excluded from swarm')
            self.publish_status()

    # -- trust view -----------------------------------------------------

    def active_members(self):
        now_ns = self.get_clock().now().nanoseconds
        alive = []
        for drone_id, last_ns in self.members_last_ns.items():
            if (now_ns - last_ns) / 1_000_000_000.0 <= MEMBER_TIMEOUT_SEC:
                alive.append(drone_id)
        return alive

    def publish_status(self):
        alive = self.active_members()
        trusted = sorted(d for d in alive if d not in self.quarantined)

        msg = SwarmStatus()
        msg.trusted_drones = trusted
        msg.quarantined_drones = sorted(self.quarantined)
        msg.stamp_ns = self.get_clock().now().nanoseconds
        self.status_publisher.publish(msg)

        if DEBUG:
            self.get_logger().info(
                f'trusted={trusted} quarantined={msg.quarantined_drones}')


def main(args=None):
    rclpy.init(args=args)
    node = ConsensusGuard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
