import rclpy
from rclpy.node import Node

from sdv_interfaces.msg import BatteryStatus
from sdv_interfaces.msg import VehicleState

DEBUG = True

class BatteryNode(Node):
    
    def __init__(self):
        super().__init__('battery_ecu')

        # =============================
        # Create Pub / Sub
        self.publisher_ = self.create_publisher(
            BatteryStatus,
            '/ecu/battery/status',
            10
        )

        self.subscription = self.create_subscription(
            VehicleState,
            '/ecu/vehicle/status',
            self.vehicle_status_callback,
            10
        )
        # =============================

        # =============================
        # Members
        self.soc = 100.0
        self.voltage = 400.0
        self.current = 10.0
        # =============================

        # =============================
        # Create Task (Periodically)
        self.timer = self.create_timer(
            1.0,
            self.Task_1000ms
        )
        # =============================

        if DEBUG :
            self.get_logger().info('Battery ECU Started')

    def Task_1000ms(self):

        msg = BatteryStatus()

        msg.soc = self.soc
        msg.voltage = self.voltage
        msg.current = self.current

        self.publisher_.publish(msg)

        if DEBUG :
            self.get_logger().info(
                f'SOC={msg.soc:.1f}%\nVOLTAGE={msg.voltage:.1f}V\nCURRENT={msg.current:.1f}A'
            )

        self.soc -= 1.0
        self.voltage -= 1.0
        self.current += 1.0

        if self.soc < 0.0:
            self.soc = 100.0
        if self.voltage < 0.0:
            self.voltage = 400.0
        if self.current < 0.0:
            self.current = 10.0
    
    def vehicle_status_callback(self, msg):
        if DEBUG:
            self.get_logger().info(
                f'Current Vehicle State = {msg.state}'
            )

def main(args=None):
    rclpy.init(args=args)

    node = BatteryNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
