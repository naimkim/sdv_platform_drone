from motor_ecu.motor_driver import SimMotorDriver
import rclpy
from rclpy.node import Node
from sdv_interfaces.msg import Heartbeat
from sdv_interfaces.msg import MotorStatus
from sdv_interfaces.msg import VehicleState

# Node Config VARs
DEBUG = False
DEBUG_MOTOR_STATUS_MSG = False
DEBUG_VEHICLE_STATE_MSG = False
TASK_USE_1MS = False
TASK_USE_10MS = False
TASK_USE_100MS = True
TASK_USE_1000MS = True
LOW_BATTERY_TARGET_LINEAR = 0.1


class MotorNode(Node):

    def __init__(self):
        super().__init__('motor_ecu')

        # =============================
        # Members
        self.declare_parameter('driver_mode', 'sim')
        self.declare_parameter('target_linear', 0.4)
        self.declare_parameter('target_angular', 0.0)

        self.driver_mode = self.get_parameter('driver_mode').value
        self.mission_target_linear = self.get_parameter('target_linear').value
        self.mission_target_angular = self.get_parameter('target_angular').value
        self.last_vehicle_state = VehicleState.INIT
        self.last_mission_active = False
        self.motor_driver = self.create_motor_driver(self.driver_mode)
        # =============================

        # =============================
        # Create Pub / Sub
        self.heart_beat_publisher = self.create_publisher(
            Heartbeat,
            '/ecu/heartbeat',
            10
        )

        self.motor_status_publisher = self.create_publisher(
            MotorStatus,
            '/ecu/motor/status',
            10
        )

        self.vehicle_state_sub = self.create_subscription(
            VehicleState,
            '/ecu/vehicle/status',
            self.vehicle_state_callback,
            10
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

        if DEBUG:
            self.get_logger().info(
                'Motor ECU Started'
            )

# ===================
# CALLBACKs
    def vehicle_state_callback(self, msg):
        self.last_vehicle_state = msg.state
        self.last_mission_active = msg.mission_active
        self.apply_vehicle_state_policy()

        if DEBUG_VEHICLE_STATE_MSG:
            self.get_logger().info(
                f'Current Vehicle State = {msg.state}, '
                f'mission_active={msg.mission_active}'
            )
# =================== End of CALLBACKs

# ===================
# Task Implementation
    def Task_100ms(self):
        self.apply_vehicle_state_policy()
        self.motor_driver.update(0.1)
        self.publish_motor_status()

    def Task_1000ms(self):
        self.publish_heartbeat()
# ===================

# ===================
# Functions
    def create_motor_driver(self, driver_mode):
        if driver_mode == 'sim':
            return SimMotorDriver()

        self.get_logger().warn(
            f'Unknown motor driver mode: {driver_mode}, fallback to sim'
        )
        return SimMotorDriver()

    def apply_vehicle_state_policy(self):
        if (
            self.last_vehicle_state == VehicleState.MISSION and
            self.last_mission_active
        ):
            self.motor_driver.set_velocity(
                self.mission_target_linear,
                self.mission_target_angular
            )
        elif (
            self.last_vehicle_state == VehicleState.LOW_BATTERY and
            self.last_mission_active
        ):
            self.motor_driver.set_velocity(
                LOW_BATTERY_TARGET_LINEAR,
                0.0
            )
        elif self.last_vehicle_state == VehicleState.EMERGENCY:
            self.motor_driver.emergency_stop()
        elif self.last_vehicle_state == VehicleState.FAULT:
            self.motor_driver.stop()
        else:
            self.motor_driver.stop()

    def publish_heartbeat(self):
        msg = Heartbeat()
        msg.ecu_name = 'motor_ecu'
        msg.timestamp = self.get_clock().now().nanoseconds

        self.heart_beat_publisher.publish(msg)

    def publish_motor_status(self):
        status = self.motor_driver.get_status()

        msg = MotorStatus()
        msg.target_linear = float(status['target_linear'])
        msg.current_linear = float(status['current_linear'])
        msg.target_angular = float(status['target_angular'])
        msg.current_angular = float(status['current_angular'])
        msg.enabled = bool(status['enabled'])

        self.motor_status_publisher.publish(msg)

        if DEBUG_MOTOR_STATUS_MSG:
            self.get_logger().info(
                f'Motor Status: target_linear={msg.target_linear:.2f}, '
                f'current_linear={msg.current_linear:.2f}, '
                f'target_angular={msg.target_angular:.2f}, '
                f'current_angular={msg.current_angular:.2f}, '
                f'enabled={msg.enabled}'
            )
# ===================


def main(args=None):
    rclpy.init(args=args)

    node = MotorNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
