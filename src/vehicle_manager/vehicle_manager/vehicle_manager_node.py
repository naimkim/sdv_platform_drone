import rclpy
from rclpy.node import Node

from sdv_interfaces.msg import BatteryStatus
from sdv_interfaces.msg import VehicleState
from enum import IntEnum

DEBUG = True

class VehicleState_e(IntEnum):
    INIT = 0
    READY = 1
    MISSION = 2
    LOW_BATTERY = 3
    FAULT = 4
    EMERGENCY = 5

class VehicleManagerNode(Node):

    def __init__(self):
        super().__init__('vehicle_manager')

        # =============================
        # Create Pub / Sub
        self.state_pub = self.create_publisher(
            VehicleState,
            '/ecu/vehicle/status',
            10
        )

        self.subscription = self.create_subscription(
            BatteryStatus,
            '/ecu/battery/status',
            self.battery_callback,
            10
        )
        # =============================
        
        # =============================
        # Members
        self.state = VehicleState_e.INIT
        # =============================
       
        # =============================
        # Create Task (Periodically)
        self.create_timer(
            1.0,
            self.Task_1000ms
        )
        # =============================        

        if DEBUG : 
            self.get_logger().info(
                'Vehicle Manager Started'
            )
    
    def battery_callback(self,msg):
        if msg.soc <= 20.0:
            if self.state == VehicleState_e.MISSION:
                self.change_state(
                    VehicleState_e.LOW_BATTERY
                )
        if DEBUG :
            self.get_logger().info(
                f'Received SOC={msg.soc:.1f}%\nReceived VOLTAGE={msg.voltage:.1f}V\nRecevice CURRENT={msg.current:.1f}A'
            )
# ===================
# STUB
# ===================
    def fault_callback(self,msg):
        self.change_state(
            VehicleState_e.FAULT
        )

    def security_callback(self,msg):
        self.change_state(
            VehicleState_e.EMERGENCY
        )    
# ===================
# END
# ===================

# ===================
# Task Implementation
    def Task_1000ms(self):
        if self.state == VehicleState_e.INIT:
            self.change_state(VehicleState_e.READY)
# ===================
    
    def publish_vehicle_state(self):
        msg = VehicleState()
        msg.state = int(self.state)
        self.state_pub.publish(msg)

    def change_state(self, new_state):
        if self.state == new_state:
            return
        if DEBUG :
            self.get_logger().info(
                f'State Change : {self.state.name} -> {new_state.name}'
            )
        self.state = new_state
        self.publish_vehicle_state()

def main(args=None):
    rclpy.init(args=args)

    node = VehicleManagerNode()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()