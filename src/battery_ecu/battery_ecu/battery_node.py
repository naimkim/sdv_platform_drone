import rclpy
from rclpy.node import Node

from sdv_interfaces.msg import BatteryStatus
from sdv_interfaces.msg import Heartbeat
from sdv_interfaces.msg import VehicleState

from datetime import datetime

DEBUG_BATTERY_ECU = False
DEBUG_VEHICLE_STATE_MSG = True
DEBUG_TASK = False

class BatteryNode(Node):
    
    def __init__(self):
        super().__init__('battery_ecu')

        # =============================
        # Members
        self.soc = 100.0
        self.voltage = 400.0
        self.current = 10.0
        if DEBUG_TASK:
            self.cnt_1ms = 0
            self.cnt_10ms = 0
            self.cnt_100ms = 0
            self.cnt_1000ms = 0
        # =============================

        # =============================
        # Create Pub / Sub
        self.heart_beat_publisher_ = self.create_publisher(
            Heartbeat,
            '/ecu/heartbeat',
            10
        )

        self.battery_status_publisher_ = self.create_publisher(
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
        # Create Task (Periodically)
        self.timer_1ms = self.create_timer(0.001, self.Task_1ms)
        self.timer_10ms = self.create_timer(0.01, self.Task_10ms)
        self.timer_100ms = self.create_timer(0.1, self.Task_100ms)
        self.timer_1000ms = self.create_timer(1.0, self.Task_1000ms)
        # =============================

        if DEBUG_BATTERY_ECU :
            self.get_logger().info('Battery ECU Started')

# ===================
# CallBacks            
    def vehicle_status_callback(self, msg):
        if DEBUG_VEHICLE_STATE_MSG:
            self.get_logger().info(
                f'Current Vehicle State = {msg.state}'
            )
# ===================

# ===================
# Task Implementation
    def Task_1ms(self):
        if DEBUG_TASK:
            self.cnt_1ms += 1
            self.get_logger().info(f'Task_1ms , executed {self.cnt_1ms}')
    def Task_10ms(self):
        if DEBUG_TASK:
            self.cnt_10ms += 1
            self.get_logger().info(f'Task_10ms , executed {self.cnt_10ms}')
    def Task_100ms(self):
        if DEBUG_TASK:
            self.cnt_100ms += 1
            self.get_logger().info(f'Task_100ms , executed {self.cnt_100ms}')
    def Task_1000ms(self):
        if DEBUG_TASK:
            self.cnt_1000ms += 1
            self.get_logger().info(f'Task_1000ms , executed {self.cnt_1000ms}')
        hb_msg = Heartbeat()
        bs_msg = BatteryStatus()

        hb_msg.ecu_name = "battery_ecu"
        hb_msg.timestamp = self.get_clock().now().nanoseconds

        bs_msg.soc = self.soc
        bs_msg.voltage = self.voltage
        bs_msg.current = self.current

        self.heart_beat_publisher_.publish(hb_msg)
        self.battery_status_publisher_.publish(bs_msg)

        if DEBUG_BATTERY_ECU :
            self.get_logger().info(
                f'SOC={bs_msg.soc:.1f}%\nVOLTAGE={bs_msg.voltage:.1f}V\nCURRENT={bs_msg.current:.1f}A'
            )
            self.get_logger().info(
                f'Node={hb_msg.ecu_name} , timestamp={hb_msg.timestamp}'
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
# ===================

# ===================    
# Functions

# ===================    

def main(args=None):
    rclpy.init(args=args)

    node = BatteryNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
