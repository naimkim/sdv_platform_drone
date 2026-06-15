import rclpy
from rclpy.node import Node

from sdv_interfaces.msg import BatteryStatus
from sdv_interfaces.msg import Heartbeat
from sdv_interfaces.msg import VehicleState
from sdv_interfaces.srv import StartMission
from enum import IntEnum

# Node Config VARs
DEBUG_VEHICLE_MANAGER = True
DEBUG_BATTERY_MSG = False
DEBUG_HEARTBEAT_MSG = True
DEBUG_TASK = False
TASK_USE_1MS = False
TASK_USE_10MS = False
TASK_USE_100MS = True
TASK_USE_1000MS = True

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
        # Members
        self.state = VehicleState_e.INIT
        self.received_initial_battery_status = False
        self.mission_duration_sec = 5.0
        self.mission_started_ns = None

        self.ecu_health = {
            "battery_ecu": {
            "required": True,
            "timeout_sec": 3.0,
            "last_seen_ns": None,
            "alive": False,
            },
            "sensor_ecu": {
            "required": True,
            "timeout_sec": 3.0,
            "last_seen_ns": None,
            "alive": False,
            },
            "motor_ecu": {
            "required": True,
            "timeout_sec": 3.0,
            "last_seen_ns": None,
            "alive": False,
            },
            "security_ecu": {
            "required": False,
            "timeout_sec": 5.0,
            "last_seen_ns": None,
            "alive": False,
            },
        }
        if DEBUG_TASK:
            self.cnt_1ms = 0
            self.cnt_10ms = 0
            self.cnt_100ms = 0
            self.cnt_1000ms = 0
        # =============================

        # =============================
        # Create Pub / Sub
        self.state_pub = self.create_publisher(
            VehicleState,
            '/ecu/vehicle/status',
            10
        )

        self.battery_subscription = self.create_subscription(
            BatteryStatus,
            '/ecu/battery/status',
            self.battery_callback,
            10
        )

        self.heartbeat_subscription = self.create_subscription(
            Heartbeat,
            '/ecu/heartbeat',
            self.heart_beat_callback,
            10
        )

        self.start_mission_service = self.create_service(
            StartMission,
            '/ecu/vehicle/start_mission',
            self.start_mission_callback
        )
        # =============================
        
        # =============================
        # Create Task (Periodically)
        if TASK_USE_1MS:
            self.timer_1ms = self.create_timer(0.001, self.Task_1ms)
        if TASK_USE_10MS:
            self.timer_10ms = self.create_timer(0.01, self.Task_10ms)
        if TASK_USE_100MS:
            self.timer_100ms = self.create_timer(0.1, self.Task_100ms)
        if TASK_USE_1000MS:
            self.timer_1000ms = self.create_timer(1.0, self.Task_1000ms)
        # =============================        

        if DEBUG_VEHICLE_MANAGER : 
            self.get_logger().info(
                'Vehicle Manager Started'
            )
# ===================
# CALLBACKs
    def battery_callback(self,msg):
        self.received_initial_battery_status = True

        if msg.soc <= 20.0:
            if self.state == VehicleState_e.MISSION:
                self.change_state(
                    VehicleState_e.LOW_BATTERY
                )
        if DEBUG_BATTERY_MSG :
            self.get_logger().info(
                f'Received SOC={msg.soc:.1f}%\nReceived VOLTAGE={msg.voltage:.1f}V\nRecevice CURRENT={msg.current:.1f}A'
            )

    def heart_beat_callback(self,msg):
        ecu_name = msg.ecu_name 

        if ecu_name not in self.ecu_health:
            self.get_logger().warn(f"Unknown ECU heartbeat: {ecu_name}")
            return

        self.ecu_health[ecu_name]["last_seen_ns"] = self.get_clock().now().nanoseconds
        self.ecu_health[ecu_name]["alive"] = True

        if DEBUG_HEARTBEAT_MSG :
            self.get_logger().info(
                f'HeartBeat Received={msg.ecu_name}, TimeStamp={msg.timestamp}'
            )

    def start_mission_callback(self, request, response):
        if self.state == VehicleState_e.READY:
            self.mission_started_ns = self.get_clock().now().nanoseconds
            self.change_state(VehicleState_e.MISSION)
            response.success = True
            response.message = 'Mission started'
            return response

        if self.state == VehicleState_e.MISSION:
            response.success = True
            response.message = 'Mission already running'
            return response

        response.success = False
        response.message = f'Cannot start mission from {self.state.name}'
        return response
# ===================
# STUB
    def fault_callback(self,msg):
        self.change_state(
            VehicleState_e.FAULT
        )

    def security_callback(self,msg):
        self.change_state(
            VehicleState_e.EMERGENCY
        )    
# =================== End of Stub
# =================== End of CALLBACKs

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

        if not self.check_heartbeat_timeout():
            return

        if self.state == VehicleState_e.INIT:
            if self.is_system_ready():
                self.change_state(VehicleState_e.READY)
        elif self.state == VehicleState_e.MISSION:
            if self.is_mission_complete():
                self.mission_started_ns = None
                self.change_state(VehicleState_e.READY)

        self.publish_vehicle_state()
# ===================

# ===================    
# Functions
    def publish_vehicle_state(self):
        msg = VehicleState()
        msg.state = int(self.state)
        self.state_pub.publish(msg)

    def change_state(self, new_state):
        if self.state == new_state:
            return
        if self.state == VehicleState_e.MISSION and new_state != VehicleState_e.MISSION:
            self.mission_started_ns = None
        if DEBUG_VEHICLE_MANAGER :
            self.get_logger().info(
                f'State Change : {self.state.name} -> {new_state.name}'
            )
        self.state = new_state
        self.publish_vehicle_state()
    
    def is_system_ready(self):
        return self.are_required_ecus_alive() and self.received_initial_battery_status
    
    def are_required_ecus_alive(self):
        for health in self.ecu_health.values():
            if health["required"] and not health["alive"]:
                return False
        return True
    
    def check_heartbeat_timeout(self):
        now_ns = self.get_clock().now().nanoseconds

        for ecu_name, health in self.ecu_health.items():
            last_seen_ns = health["last_seen_ns"]
            if last_seen_ns is None:
                if health["required"]:
                    return False
                continue
            elapsed_sec = (now_ns - last_seen_ns) / 1_000_000_000.0

            if elapsed_sec > health["timeout_sec"]:
                health["alive"] = False

                if health["required"]:
                    if DEBUG_HEARTBEAT_MSG:
                        self.get_logger().error(
                            f'Required ECU timeout : {ecu_name}, elapsed={elapsed_sec:.1f}s'
                        )
                    self.change_state(VehicleState_e.FAULT)
                    return False
                if DEBUG_HEARTBEAT_MSG:
                    self.get_logger().warn(
                        f"Optional ECU timeout: {ecu_name}, elapsed={elapsed_sec:.1f}s"
                    )
        return True

    def is_mission_complete(self):
        if self.mission_started_ns is None:
            return False

        now_ns = self.get_clock().now().nanoseconds
        elapsed_sec = (now_ns - self.mission_started_ns) / 1_000_000_000.0
        return elapsed_sec >= self.mission_duration_sec
# ===================

def main(args=None):
    rclpy.init(args=args)

    node = VehicleManagerNode()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
