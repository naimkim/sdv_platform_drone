import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode

from drone_offboard.pid import PID


# Node Config VARs
DEBUG = False
CONTROL_RATE_HZ = 20.0          # >2 Hz is required to hold OFFBOARD
SETPOINT_WARMUP_TICKS = 30      # stream setpoints before requesting OFFBOARD
REQUEST_THROTTLE_SEC = 1.0      # rate-limit arm / set_mode calls

ARRIVAL_RADIUS = 0.4            # m, advance to next waypoint
MAX_HORIZONTAL_SPEED = 2.0     # m/s
MAX_VERTICAL_SPEED = 1.0       # m/s

# Default PID gains (position error [m] -> velocity setpoint [m/s]).
# Tuned conservatively; override from the launch file to demonstrate tuning.
DEFAULT_KP_XY = 0.9
DEFAULT_KI_XY = 0.05
DEFAULT_KD_XY = 0.15
DEFAULT_KP_Z = 1.2
DEFAULT_KI_Z = 0.1
DEFAULT_KD_Z = 0.0

# Flight phases.
WAIT_FCU = 'WAIT_FCU'
PREFLIGHT = 'PREFLIGHT'
MISSION = 'MISSION'
LANDING = 'LANDING'
DONE = 'DONE'

# PositionTarget mask: ignore position + acceleration + yaw_rate, use velocity
# and yaw only.
VELOCITY_YAW_MASK = (
    PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY |
    PositionTarget.IGNORE_PZ | PositionTarget.IGNORE_AFX |
    PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
    PositionTarget.IGNORE_YAW_RATE)

DEFAULT_WAYPOINTS = [
    0.0, 0.0, 3.0,   # take off straight up
    5.0, 0.0, 3.0,   # square leg 1
    5.0, 5.0, 3.0,   # leg 2
    0.0, 5.0, 3.0,   # leg 3
    0.0, 0.0, 3.0,   # return over home
]


class OffboardController(Node):
    """PX4 Offboard waypoint follower over MAVROS.

    Streams velocity setpoints derived from a per-axis PID on position error,
    which is what makes the control tuning observable. The flight phase machine
    handles connection, OFFBOARD engagement, arming, the waypoint mission and an
    automatic land.

    Note on frames: MAVROS expects ENU values on setpoint_raw/local even when
    coordinate_frame is FRAME_LOCAL_NED, and reports local_position/pose in ENU.
    Everything here is therefore ENU (x=East, y=North, z=Up).
    """

    def __init__(self):
        super().__init__('drone_offboard')

        self.declare_parameter('waypoints', DEFAULT_WAYPOINTS)
        self.declare_parameter('kp_xy', DEFAULT_KP_XY)
        self.declare_parameter('ki_xy', DEFAULT_KI_XY)
        self.declare_parameter('kd_xy', DEFAULT_KD_XY)
        self.declare_parameter('kp_z', DEFAULT_KP_Z)
        self.declare_parameter('ki_z', DEFAULT_KI_Z)
        self.declare_parameter('kd_z', DEFAULT_KD_Z)

        self.waypoints = self.load_waypoints()
        self.pid_x = self.make_xy_pid()
        self.pid_y = self.make_xy_pid()
        self.pid_z = PID(
            self.get_parameter('kp_z').value,
            self.get_parameter('ki_z').value,
            self.get_parameter('kd_z').value,
            output_limit=MAX_VERTICAL_SPEED, integral_limit=1.0)

        self.state = State()
        self.current = None             # (x, y, z) ENU
        self.phase = WAIT_FCU
        self.wp_index = 0
        self.setpoint_ticks = 0
        self.last_request_ns = 0
        self.prev_tick_ns = None

        self.setpoint_publisher = self.create_publisher(
            PositionTarget, '/mavros/setpoint_raw/local', 10)

        self.create_subscription(
            State, '/mavros/state', self.state_callback, 10)
        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self.pose_callback, qos_profile_sensor_data)

        self.arming_client = self.create_client(
            CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(
            SetMode, '/mavros/set_mode')

        self.control_timer = self.create_timer(
            1.0 / CONTROL_RATE_HZ, self.control_tick)

        self.get_logger().info(
            f'Offboard controller started with {len(self.waypoints)} '
            f'waypoint(s)')

    # -- setup helpers --------------------------------------------------

    def make_xy_pid(self):
        return PID(
            self.get_parameter('kp_xy').value,
            self.get_parameter('ki_xy').value,
            self.get_parameter('kd_xy').value,
            output_limit=MAX_HORIZONTAL_SPEED, integral_limit=2.0)

    def load_waypoints(self):
        flat = list(self.get_parameter('waypoints').value)
        if not flat or len(flat) % 3 != 0:
            self.get_logger().warn(
                'Invalid waypoints parameter; using default square mission')
            flat = DEFAULT_WAYPOINTS
        return [tuple(flat[i:i + 3]) for i in range(0, len(flat), 3)]

    # -- subscriptions --------------------------------------------------

    def state_callback(self, msg):
        self.state = msg

    def pose_callback(self, msg):
        p = msg.pose.position
        self.current = (p.x, p.y, p.z)

    # -- main loop ------------------------------------------------------

    def control_tick(self):
        now_ns = self.get_clock().now().nanoseconds
        dt = 0.0 if self.prev_tick_ns is None \
            else (now_ns - self.prev_tick_ns) / 1e9
        self.prev_tick_ns = now_ns

        # A setpoint must be streamed every tick to keep OFFBOARD alive.
        vx, vy, vz = self.compute_velocity(dt)
        self.publish_velocity(vx, vy, vz)
        self.setpoint_ticks += 1

        if self.phase == WAIT_FCU:
            self.tick_wait_fcu()
        elif self.phase == PREFLIGHT:
            self.tick_preflight()
        elif self.phase == MISSION:
            self.tick_mission()
        elif self.phase == LANDING:
            self.tick_landing()

    def compute_velocity(self, dt):
        if self.phase != MISSION or self.current is None:
            return 0.0, 0.0, 0.0

        tx, ty, tz = self.waypoints[self.wp_index]
        cx, cy, cz = self.current
        vx = self.pid_x.update(tx - cx, dt)
        vy = self.pid_y.update(ty - cy, dt)
        vz = self.pid_z.update(tz - cz, dt)

        # Clamp the horizontal vector magnitude (per-axis limits aren't enough).
        speed = math.hypot(vx, vy)
        if speed > MAX_HORIZONTAL_SPEED:
            scale = MAX_HORIZONTAL_SPEED / speed
            vx, vy = vx * scale, vy * scale
        return vx, vy, vz

    def publish_velocity(self, vx, vy, vz):
        msg = PositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.type_mask = VELOCITY_YAW_MASK
        msg.velocity.x = float(vx)
        msg.velocity.y = float(vy)
        msg.velocity.z = float(vz)
        msg.yaw = 0.0
        self.setpoint_publisher.publish(msg)

    # -- phase handlers -------------------------------------------------

    def tick_wait_fcu(self):
        if self.state.connected:
            self.get_logger().info('FCU connected; streaming setpoints')
            self.phase = PREFLIGHT

    def tick_preflight(self):
        # Stream a warmup window of setpoints before engaging OFFBOARD.
        if self.setpoint_ticks < SETPOINT_WARMUP_TICKS:
            return
        if not self.throttle_ready():
            return

        if self.state.mode != 'OFFBOARD':
            self.request_mode('OFFBOARD')
        elif not self.state.armed:
            self.request_arm(True)
        else:
            self.get_logger().info('Armed and in OFFBOARD; starting mission')
            self.reset_pids()
            self.wp_index = 0
            self.phase = MISSION

    def tick_mission(self):
        if self.current is None:
            return
        tx, ty, tz = self.waypoints[self.wp_index]
        cx, cy, cz = self.current
        dist = math.dist((tx, ty, tz), (cx, cy, cz))

        if dist <= ARRIVAL_RADIUS:
            if self.wp_index + 1 < len(self.waypoints):
                self.wp_index += 1
                self.reset_pids()
                self.get_logger().info(
                    f'Reached waypoint, advancing to #{self.wp_index} '
                    f'{self.waypoints[self.wp_index]}')
            else:
                self.get_logger().info('Final waypoint reached; landing')
                self.phase = LANDING

    def tick_landing(self):
        if self.state.mode != 'AUTO.LAND':
            if self.throttle_ready():
                self.request_mode('AUTO.LAND')
        elif not self.state.armed:
            self.get_logger().info('Landed and disarmed; mission complete')
            self.phase = DONE

    # -- service plumbing ----------------------------------------------

    def throttle_ready(self):
        now_ns = self.get_clock().now().nanoseconds
        if (now_ns - self.last_request_ns) / 1e9 < REQUEST_THROTTLE_SEC:
            return False
        self.last_request_ns = now_ns
        return True

    def request_mode(self, mode):
        if not self.set_mode_client.service_is_ready():
            return
        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = mode
        self.set_mode_client.call_async(req)
        self.get_logger().info(f'Requested mode {mode}')

    def request_arm(self, value):
        if not self.arming_client.service_is_ready():
            return
        req = CommandBool.Request()
        req.value = value
        self.arming_client.call_async(req)
        self.get_logger().info(f'Requested arming={value}')

    def reset_pids(self):
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_z.reset()


def main(args=None):
    rclpy.init(args=args)
    node = OffboardController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
