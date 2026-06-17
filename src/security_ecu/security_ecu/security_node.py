import rclpy
from rclpy.node import Node

from sdv_interfaces.msg import Heartbeat


# Node Config VARs
DEBUG = False
TASK_USE_1000MS = True


class SecurityNode(Node):

    def __init__(self):
        super().__init__('security_ecu')

        self.heart_beat_publisher = self.create_publisher(
            Heartbeat,
            '/ecu/heartbeat',
            10
        )

        if TASK_USE_1000MS:
            self.timer_1000ms = self.create_timer(1.0, self.Task_1000ms)

        if DEBUG:
            self.get_logger().info('Security ECU Started')

    def Task_1000ms(self):
        self.publish_heartbeat()

    def publish_heartbeat(self):
        msg = Heartbeat()
        msg.ecu_name = 'security_ecu'
        msg.timestamp = self.get_clock().now().nanoseconds

        self.heart_beat_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = SecurityNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
