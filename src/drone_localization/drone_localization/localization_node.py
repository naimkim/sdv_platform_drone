import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from drone_interfaces.msg import LocalizationStatus

from drone_localization.ekf import KalmanFilter, innovation_distance
from drone_localization.gps_monitor import GPSIntegrityMonitor


# Node Config VARs
DEBUG = False
PREDICT_RATE_HZ = 30.0
GPS_TIMEOUT_SEC = 1.0           # no GPS for this long -> treated as denied

PROCESS_NOISE = 0.5
VIO_NOISE = 0.04               # m^2, VIO position variance (tight)
GPS_NOISE = 1.0                # m^2, GPS position variance (loose)

GPS_GATE = 2.0                 # m, innovation gate for spoof/divergence
GPS_TRIP_COUNT = 3
GPS_RECOVER_GATE = 1.0
GPS_RECOVER_COUNT = 15


class LocalizationNode(Node):
    """Fuses VIO and GPS into a single odometry estimate, and falls back to
    VIO-only navigation when the GPS integrity monitor rejects GPS.

    Topics:
      sub  /vio/odom              (nav_msgs/Odometry)   VIO position
      sub  /gps/pose              (geometry_msgs/PoseStamped) local GPS fix
      pub  /drone/odom            (nav_msgs/Odometry)   fused estimate
      pub  /drone/localization_status (LocalizationStatus)
    """

    def __init__(self):
        super().__init__('drone_localization')

        self.kf = KalmanFilter(process_noise=PROCESS_NOISE)
        self.monitor = GPSIntegrityMonitor(
            gate=GPS_GATE, trip_count=GPS_TRIP_COUNT,
            recover_gate=GPS_RECOVER_GATE, recover_count=GPS_RECOVER_COUNT)

        self.last_gps_ns = None
        self.prev_tick_ns = None

        self.odom_publisher = self.create_publisher(
            Odometry, '/drone/odom', 10)
        self.status_publisher = self.create_publisher(
            LocalizationStatus, '/drone/localization_status', 10)

        self.create_subscription(
            Odometry, '/vio/odom', self.vio_callback, qos_profile_sensor_data)
        self.create_subscription(
            PoseStamped, '/gps/pose', self.gps_callback,
            qos_profile_sensor_data)

        self.timer = self.create_timer(
            1.0 / PREDICT_RATE_HZ, self.predict_tick)

        self.get_logger().info('Localization node started (VIO + GPS fusion)')

    # -- measurement callbacks -----------------------------------------

    def vio_callback(self, msg):
        p = msg.pose.pose.position
        # VIO is always fused; it is the GPS-denied fallback source.
        self.kf.update_position((p.x, p.y, p.z), VIO_NOISE)

    def gps_callback(self, msg):
        self.last_gps_ns = self.get_clock().now().nanoseconds
        p = msg.pose.position
        position = (p.x, p.y, p.z)

        innovation = self.kf.innovation(position)
        dist = innovation_distance(innovation)
        was_trusted = self.monitor.trusted
        trusted, reason = self.monitor.update(dist, gps_available=True)

        if trusted:
            self.kf.update_position(position, GPS_NOISE)
        elif was_trusted and not trusted:
            self.get_logger().warn(reason)

    # -- prediction + output -------------------------------------------

    def predict_tick(self):
        now_ns = self.get_clock().now().nanoseconds
        dt = 0.0 if self.prev_tick_ns is None \
            else (now_ns - self.prev_tick_ns) / 1e9
        self.prev_tick_ns = now_ns

        self.kf.predict(dt)
        self.check_gps_timeout(now_ns)
        self.publish_odom(now_ns)
        self.publish_status(now_ns)

    def check_gps_timeout(self, now_ns):
        stale = (self.last_gps_ns is None or
                 (now_ns - self.last_gps_ns) / 1e9 > GPS_TIMEOUT_SEC)
        if stale and self.monitor.trusted:
            _, reason = self.monitor.update(
                self.monitor.last_innovation, gps_available=False)
            self.get_logger().warn(f'GPS-denied: {reason}')

    def publish_odom(self, now_ns):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.child_frame_id = 'base_link'

        pos = self.kf.position
        vel = self.kf.velocity
        msg.pose.pose.position.x = float(pos[0])
        msg.pose.pose.position.y = float(pos[1])
        msg.pose.pose.position.z = float(pos[2])
        msg.pose.pose.orientation.w = 1.0
        msg.twist.twist.linear.x = float(vel[0])
        msg.twist.twist.linear.y = float(vel[1])
        msg.twist.twist.linear.z = float(vel[2])

        # Fill the position block of the 6x6 row-major pose covariance.
        cov = self.kf.position_covariance
        pose_cov = [0.0] * 36
        for r in range(3):
            for c in range(3):
                pose_cov[r * 6 + c] = float(cov[r, c])
        msg.pose.covariance = pose_cov

        self.odom_publisher.publish(msg)

    def publish_status(self, now_ns):
        msg = LocalizationStatus()
        msg.gps_trusted = self.monitor.trusted
        msg.mode = LocalizationStatus.MODE_GPS_FUSED if self.monitor.trusted \
            else LocalizationStatus.MODE_GPS_DENIED
        msg.reason = self.monitor.reason
        msg.gps_innovation = float(self.monitor.last_innovation)
        msg.stamp_ns = now_ns
        self.status_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
