import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import Point, TransformStamped
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from swarm_interfaces.msg import SecurityAlert
from swarm_interfaces.msg import SignedPose
from swarm_interfaces.msg import SwarmStatus

from tf2_ros import StaticTransformBroadcaster


# Node Config VARs
DEBUG = False
FRAME_ID = 'map'
PUBLISH_RATE_HZ = 10.0
AREA_SIZE = 10.0                # m, must match swarm_agent
POSE_STALE_SEC = 3.0           # drop a drone marker if silent this long
ALERT_HIGHLIGHT_SEC = 3.0      # how long a suspect glows orange
BANNER_HOLD_SEC = 4.0          # how long the alert banner stays up

# Status colors (r, g, b)
COLOR_TRUSTED = (0.10, 0.80, 0.20)
COLOR_QUARANTINED = (0.90, 0.10, 0.10)
COLOR_ALERTING = (1.00, 0.55, 0.00)
COLOR_UNKNOWN = (0.45, 0.55, 0.95)
COLOR_AREA = (0.55, 0.55, 0.55)
COLOR_SECTOR = (0.30, 0.30, 0.40)


def color(rgb, a=1.0):
    c = ColorRGBA()
    c.r, c.g, c.b = float(rgb[0]), float(rgb[1]), float(rgb[2])
    c.a = float(a)
    return c


class SwarmViz(Node):
    """Translates the (custom) swarm bus into RViz MarkerArray markers.

    RViz cannot render SignedPose/SwarmStatus directly, so this monitor node
    fuses pose, trust view and alerts into a single MarkerArray:
      * a sphere per drone, colored by trust state (green/orange/red),
      * the search-area boundary and the current sector decomposition,
      * a text banner echoing the latest IDS alert.
    """

    def __init__(self):
        super().__init__('swarm_viz')

        self.poses = {}             # drone_id -> (x, y, z)
        self.pose_ns = {}           # drone_id -> last update ns
        self.id_index = {}          # drone_id -> stable marker index
        self.trusted = set()
        self.quarantined = set()
        self.alert_until = {}       # drone_id -> ns when highlight expires
        self.banner_text = ''
        self.banner_until = 0

        self.marker_publisher = self.create_publisher(
            MarkerArray, '/swarm/markers', 10)

        self.create_subscription(
            SignedPose, '/swarm/pose', self.pose_callback, 50)
        self.create_subscription(
            SwarmStatus, '/swarm/status', self.status_callback, 10)
        self.create_subscription(
            SecurityAlert, '/swarm/security_alert', self.alert_callback, 10)

        # Register the fixed frame so RViz has a valid TF root.
        self.static_tf = StaticTransformBroadcaster(self)
        self.publish_static_frame()

        self.timer = self.create_timer(
            1.0 / PUBLISH_RATE_HZ, self.publish_markers)

        self.get_logger().info('Swarm viz started (markers on /swarm/markers)')

    def publish_static_frame(self):
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = FRAME_ID
        tf.child_frame_id = 'swarm_origin'
        tf.transform.rotation.w = 1.0
        self.static_tf.sendTransform(tf)

    # -- intake ---------------------------------------------------------

    def pose_callback(self, msg):
        self.poses[msg.drone_id] = (msg.x, msg.y, msg.z)
        self.pose_ns[msg.drone_id] = self.get_clock().now().nanoseconds
        if msg.drone_id not in self.id_index:
            self.id_index[msg.drone_id] = len(self.id_index)

    def status_callback(self, msg):
        self.trusted = set(msg.trusted_drones)
        self.quarantined = set(msg.quarantined_drones)

    def alert_callback(self, msg):
        now_ns = self.get_clock().now().nanoseconds
        self.alert_until[msg.suspect_id] = \
            now_ns + int(ALERT_HIGHLIGHT_SEC * 1e9)
        self.banner_text = f'IDS: [{msg.alert_type}] {msg.suspect_id}'
        self.banner_until = now_ns + int(BANNER_HOLD_SEC * 1e9)

    # -- rendering ------------------------------------------------------

    def drone_color(self, drone_id, now_ns):
        if drone_id in self.quarantined:
            return COLOR_QUARANTINED
        if self.alert_until.get(drone_id, 0) > now_ns:
            return COLOR_ALERTING
        if drone_id in self.trusted:
            return COLOR_TRUSTED
        return COLOR_UNKNOWN

    def active_drones(self, now_ns):
        live = []
        for drone_id, ns in self.pose_ns.items():
            if (now_ns - ns) / 1e9 <= POSE_STALE_SEC:
                live.append(drone_id)
        return live

    def publish_markers(self):
        now_ns = self.get_clock().now().nanoseconds
        markers = MarkerArray()

        markers.markers.append(self.area_boundary())
        markers.markers.extend(self.sector_dividers())

        for drone_id in self.active_drones(now_ns):
            x, y, z = self.poses[drone_id]
            rgb = self.drone_color(drone_id, now_ns)
            idx = self.id_index[drone_id]
            markers.markers.append(self.sphere(idx, x, y, z, rgb))
            markers.markers.append(self.label(idx, drone_id, x, y, z))

        markers.markers.append(self.banner(now_ns))

        self.marker_publisher.publish(markers)

    def base(self, ns, mid, mtype):
        m = Marker()
        m.header.frame_id = FRAME_ID
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    def area_boundary(self):
        m = self.base('area', 0, Marker.LINE_STRIP)
        m.scale.x = 0.08
        m.color = color(COLOR_AREA)
        corners = [(0, 0), (AREA_SIZE, 0), (AREA_SIZE, AREA_SIZE),
                   (0, AREA_SIZE), (0, 0)]
        for cx, cy in corners:
            m.points.append(Point(x=float(cx), y=float(cy), z=0.0))
        return m

    def sector_dividers(self):
        out = []
        n = max(1, len(self.trusted))
        strip = AREA_SIZE / n
        for i in range(1, n):
            m = self.base('sectors', 10 + i, Marker.LINE_STRIP)
            m.scale.x = 0.04
            m.color = color(COLOR_SECTOR, 0.8)
            x = strip * i
            m.points.append(Point(x=float(x), y=0.0, z=0.0))
            m.points.append(Point(x=float(x), y=float(AREA_SIZE), z=0.0))
            out.append(m)
        # Clear leftover divider ids when the trusted set shrinks.
        for i in range(n, 6):
            stale = self.base('sectors', 10 + i, Marker.LINE_STRIP)
            stale.action = Marker.DELETE
            out.append(stale)
        return out

    def sphere(self, idx, x, y, z, rgb):
        m = self.base('drones', 100 + idx, Marker.SPHERE)
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.position.z = float(z)
        m.scale.x = m.scale.y = m.scale.z = 0.7
        m.color = color(rgb)
        return m

    def label(self, idx, drone_id, x, y, z):
        m = self.base('labels', 200 + idx, Marker.TEXT_VIEW_FACING)
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.position.z = float(z) + 0.7
        m.scale.z = 0.5
        m.color = color((1.0, 1.0, 1.0))
        m.text = drone_id
        return m

    def banner(self, now_ns):
        m = self.base('banner', 900, Marker.TEXT_VIEW_FACING)
        m.pose.position.x = AREA_SIZE / 2.0
        m.pose.position.y = AREA_SIZE + 1.2
        m.pose.position.z = 1.0
        m.scale.z = 0.7
        if self.banner_until > now_ns and self.banner_text:
            m.color = color(COLOR_ALERTING)
            m.text = self.banner_text
        else:
            m.color = color(COLOR_TRUSTED)
            m.text = 'swarm nominal'
        return m


def main(args=None):
    rclpy.init(args=args)
    node = SwarmViz()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
