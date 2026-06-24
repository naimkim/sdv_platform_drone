import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Odometry
from vision_msgs.msg import Detection2DArray

from drone_perception.detection_geometry import (
    estimate_distance,
    focal_length_px,
    obstacle_offset,
)


# Node Config VARs
DEBUG = False
IMAGE_WIDTH = 640
HFOV_DEG = 90.0
DEFAULT_OBJECT_HEIGHT_M = 1.0    # assumed real height for range-from-bbox
MAX_RANGE_M = 8.0               # ignore detections estimated beyond this


class PerceptionNode(Node):
    """Bridges upstream YOLO detections to navigation obstacles.

    The detector (YOLO + TensorRT on Jetson) publishes vision_msgs/Detection2DArray;
    this node estimates each object's range from its bounding box, projects it to
    a relative position, transforms it to the world frame using the fused
    odometry, and republishes the set as a PoseArray the avoidance node consumes.

    Frame assumption (consistent with Phase 1/2): the drone holds yaw = 0, so the
    body x/y axes align with world ENU; no rotation is applied.

    Topics:
      sub  /detections        (vision_msgs/Detection2DArray)  upstream YOLO
      sub  /drone/odom        (nav_msgs/Odometry)             drone pose
      pub  /perception/obstacles (geometry_msgs/PoseArray)    world obstacles
    """

    def __init__(self):
        super().__init__('drone_perception')

        self.declare_parameter('image_width', IMAGE_WIDTH)
        self.declare_parameter('hfov_deg', HFOV_DEG)
        self.declare_parameter('object_height_m', DEFAULT_OBJECT_HEIGHT_M)
        self.declare_parameter('max_range_m', MAX_RANGE_M)

        self.image_width = int(self.get_parameter('image_width').value)
        self.hfov_rad = math.radians(float(self.get_parameter('hfov_deg').value))
        self.object_height = float(self.get_parameter('object_height_m').value)
        self.max_range = float(self.get_parameter('max_range_m').value)
        self.focal_px = focal_length_px(self.image_width, self.hfov_rad)

        self.position = (0.0, 0.0)

        self.obstacle_publisher = self.create_publisher(
            PoseArray, '/perception/obstacles', 10)

        self.create_subscription(
            Detection2DArray, '/detections', self.detection_callback,
            qos_profile_sensor_data)
        self.create_subscription(
            Odometry, '/drone/odom', self.odom_callback, qos_profile_sensor_data)

        self.get_logger().info(
            f'Perception bridge started (f={self.focal_px:.1f}px, '
            f'hfov={math.degrees(self.hfov_rad):.0f}deg)')

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        self.position = (p.x, p.y)

    def detection_callback(self, msg):
        out = PoseArray()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'map'

        px, py = self.position
        for det in msg.detections:
            u, size_y = self.read_bbox(det.bbox)
            if u is None:
                continue
            distance = estimate_distance(
                self.object_height, size_y, self.focal_px)
            if not math.isfinite(distance) or distance > self.max_range:
                continue

            dx, dy = obstacle_offset(
                u, self.image_width, self.hfov_rad, distance)
            # yaw = 0: body x/y align with world ENU.
            pose = Pose()
            pose.position.x = px + dx
            pose.position.y = py + dy
            pose.orientation.w = 1.0
            out.poses.append(pose)

        self.obstacle_publisher.publish(out)
        if DEBUG:
            self.get_logger().info(f'Published {len(out.poses)} obstacle(s)')

    @staticmethod
    def read_bbox(bbox):
        """Read (center_u, size_y) from a BoundingBox2D across msg variants."""
        center = bbox.center
        # vision_msgs Pose2D has either `x`/`y` directly or a `position` member.
        u = getattr(center, 'x', None)
        if u is None:
            position = getattr(center, 'position', None)
            if position is None:
                return None, None
            u = position.x
        return float(u), float(bbox.size_y)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
