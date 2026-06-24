import math

import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from drone_avoidance.potential_field import avoidance_velocity


# Node Config VARs
DEBUG = False
COMMAND_RATE_HZ = 20.0
ARRIVAL_RADIUS = 0.4

MAX_SPEED = 1.5
ATTRACT_GAIN = 1.0
SLOW_RADIUS = 1.5
INFLUENCE_RADIUS = 2.5      # m, obstacles closer than this repel
REPULSE_GAIN = 1.2
MIN_CLEARANCE = 0.3
MAX_OBSTACLE_POINTS = 60    # cap beams turned into repulsors per cycle


class AvoidanceNode(Node):
    """Reactive potential-field obstacle avoidance.

    Consumes the fused odometry (from drone_localization) and a LaserScan, and
    publishes a horizontal velocity command that pulls toward the goal while
    pushing away from obstacles. Designed to sit between localization and the
    Offboard controller.

    Frame assumption: the drone holds yaw = 0 (as the Phase 1 controller does),
    so scan angles and world ENU axes are aligned and no rotation is needed.

    Topics:
      sub  /drone/odom        (nav_msgs/Odometry)        current pose
      sub  /scan              (sensor_msgs/LaserScan)    obstacles
      sub  /goal_pose         (geometry_msgs/PoseStamped) optional goal update
      pub  /drone/avoidance_cmd (geometry_msgs/TwistStamped) velocity command
    """

    def __init__(self):
        super().__init__('drone_avoidance')

        self.declare_parameter('goal_x', 5.0)
        self.declare_parameter('goal_y', 5.0)
        self.goal = np.array([
            float(self.get_parameter('goal_x').value),
            float(self.get_parameter('goal_y').value),
        ])

        self.position = None        # (x, y)
        self.obstacles = []         # list of (rel_x, rel_y)

        self.cmd_publisher = self.create_publisher(
            TwistStamped, '/drone/avoidance_cmd', 10)

        self.create_subscription(
            Odometry, '/drone/odom', self.odom_callback,
            qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)

        self.timer = self.create_timer(
            1.0 / COMMAND_RATE_HZ, self.command_tick)

        self.get_logger().info(
            f'Avoidance node started, goal=({self.goal[0]:.1f}, '
            f'{self.goal[1]:.1f})')

    # -- subscriptions --------------------------------------------------

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        self.position = np.array([p.x, p.y])

    def goal_callback(self, msg):
        self.goal = np.array([msg.pose.position.x, msg.pose.position.y])
        self.get_logger().info(
            f'New goal ({self.goal[0]:.1f}, {self.goal[1]:.1f})')

    def scan_callback(self, msg):
        # Convert in-range beams into relative obstacle positions (yaw aligned).
        obstacles = []
        angle = msg.angle_min
        for r in msg.ranges:
            if math.isfinite(r) and msg.range_min <= r < INFLUENCE_RADIUS:
                obstacles.append((r * math.cos(angle), r * math.sin(angle)))
            angle += msg.angle_increment
        # Keep the closest subset to bound the work per cycle.
        obstacles.sort(key=lambda o: o[0] * o[0] + o[1] * o[1])
        self.obstacles = obstacles[:MAX_OBSTACLE_POINTS]

    # -- control loop ---------------------------------------------------

    def command_tick(self):
        if self.position is None:
            return

        goal_vec = self.goal - self.position
        if float(np.linalg.norm(goal_vec)) <= ARRIVAL_RADIUS:
            self.publish_cmd(np.zeros(2))
            return

        velocity = avoidance_velocity(
            goal_vec, self.obstacles,
            max_speed=MAX_SPEED, attract_gain=ATTRACT_GAIN,
            slow_radius=SLOW_RADIUS, influence_radius=INFLUENCE_RADIUS,
            repulse_gain=REPULSE_GAIN, min_clearance=MIN_CLEARANCE)
        self.publish_cmd(velocity)

    def publish_cmd(self, velocity):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.twist.linear.x = float(velocity[0])
        msg.twist.linear.y = float(velocity[1])
        msg.twist.linear.z = 0.0
        self.cmd_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AvoidanceNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
