from enum import IntEnum

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sdv_interfaces.action import ReturnHome
from sdv_interfaces.msg import BatteryStatus
from sdv_interfaces.msg import DiagnosticEvent
from sdv_interfaces.msg import Heartbeat
from sdv_interfaces.msg import ObstacleInfo
from sdv_interfaces.msg import SecurityEvent
from sdv_interfaces.msg import VehicleState
from sdv_interfaces.srv import CompleteMission
from sdv_interfaces.srv import ResetEmergency
from sdv_interfaces.srv import StartMission


# Node Config VARs
DEBUG_VEHICLE_MANAGER = True
DEBUG_BATTERY_MSG = False
DEBUG_HEARTBEAT_MSG = False
DEBUG_OBSTACLE_MSG = False
DEBUG_TASK = False
TASK_USE_1MS = False
TASK_USE_10MS = False
TASK_USE_100MS = True
TASK_USE_1000MS = True
LOW_BATTERY_ENTER_SOC = 20.0
LOW_BATTERY_RECOVER_SOC = 25.0
LOW_BATTERY_RECOVER_HOLD_SEC = 5.0
DIAGNOSTIC_SEVERITY_ERROR = 2


class VehicleState_e(IntEnum):
    INIT = 0
    READY = 1
    MISSION = 2
    LOW_BATTERY = 3
    FAULT = 4
    EMERGENCY = 5
    MRM = 6


class VehicleManagerNode(Node):

    def __init__(self):
        super().__init__('vehicle_manager')
        # =============================
        # Members
        self.state = VehicleState_e.INIT
        self.received_initial_battery_status = False
        self.mission_active = False
        self.declare_parameter('mission_duration_sec', 12.0)
        self.mission_duration_sec = float(
            self.get_parameter('mission_duration_sec').value
        )
        self.mission_started_ns = None
        self.low_battery_recovery_started_ns = None
        self.obstacle_detected = False
        self.obstacle_stop_active = False
        self.obstacle_pause_started_ns = None
        self.resume_after_obstacle_clear = False
        self.mrm_return_goal_active = False
        self.mrm_return_retry_count = 0
        self.mrm_return_retry_timer = None

        self.ecu_health = {
            'battery_ecu': {
                'required': True,
                'timeout_sec': 3.0,
                'last_seen_ns': None,
                'alive': False,
            },
            'sensor_ecu': {
                'required': True,
                'timeout_sec': 3.0,
                'last_seen_ns': None,
                'alive': False,
            },
            'motor_ecu': {
                'required': True,
                'timeout_sec': 3.0,
                'last_seen_ns': None,
                'alive': False,
            },
            'diagnostics_ecu': {
                'required': False,
                'timeout_sec': 5.0,
                'last_seen_ns': None,
                'alive': False,
            },
            'security_ecu': {
                'required': False,
                'timeout_sec': 5.0,
                'last_seen_ns': None,
                'alive': False,
            },
        }
        # =============================
        if DEBUG_TASK:
            self.cnt_1ms = 0
            self.cnt_10ms = 0
            self.cnt_100ms = 0
            self.cnt_1000ms = 0
        # =============================
        # Topic Pub/Sub
        self.heart_beat_publisher = self.create_publisher(
            Heartbeat,
            '/ecu/heartbeat',
            10
        )

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

        self.diagnostic_event_subscription = self.create_subscription(
            DiagnosticEvent,
            '/ecu/diagnostics/event',
            self.diagnostic_event_callback,
            10
        )

        self.obstacle_subscription = self.create_subscription(
            ObstacleInfo,
            '/ecu/obstacle/info',
            self.obstacle_callback,
            10
        )

        self.security_event_subscription = self.create_subscription(
            SecurityEvent,
            '/ecu/security/event',
            self.security_event_callback,
            10
        )
        # =============================

        # =============================
        # Service
        self.start_mission_service = self.create_service(
            StartMission,
            '/ecu/vehicle/start_mission',
            self.start_mission_callback
        )

        self.reset_emergency_service = self.create_service(
            ResetEmergency,
            '/ecu/vehicle/reset_emergency',
            self.reset_emergency_callback
        )

        self.complete_mission_service = self.create_service(
            CompleteMission,
            '/ecu/vehicle/complete_mission',
            self.complete_mission_callback
        )

        self.return_home_client = ActionClient(
            self,
            ReturnHome,
            '/return_home'
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
        if DEBUG_VEHICLE_MANAGER:
            self.get_logger().info('Vehicle Manager Started')

# =============================
# Callbacks
    def battery_callback(self, msg):
        self.received_initial_battery_status = True

        if msg.soc <= LOW_BATTERY_ENTER_SOC:
            if (
                self.state == VehicleState_e.MISSION or
                self.state == VehicleState_e.READY
            ):
                self.start_low_battery_mrm()
            self.low_battery_recovery_started_ns = None
        elif self.state == VehicleState_e.LOW_BATTERY:
            self.update_low_battery_recovery(msg.soc)

        if DEBUG_BATTERY_MSG:
            self.get_logger().info(
                f'Received SOC={msg.soc:.1f}%\n'
                f'Received VOLTAGE={msg.voltage:.1f}V\n'
                f'Received CURRENT={msg.current:.1f}A'
            )

    def heart_beat_callback(self, msg):
        ecu_name = msg.ecu_name

        if ecu_name == 'vehicle_manager':
            return

        if ecu_name not in self.ecu_health:
            self.get_logger().warn(f'Unknown ECU heartbeat: {ecu_name}')
            return

        self.ecu_health[ecu_name]['last_seen_ns'] = (
            self.get_clock().now().nanoseconds
        )
        self.ecu_health[ecu_name]['alive'] = True

        if DEBUG_HEARTBEAT_MSG:
            self.get_logger().info(
                f'HeartBeat Received={msg.ecu_name}, '
                f'TimeStamp={msg.timestamp}'
            )

    def diagnostic_event_callback(self, msg):
        if msg.severity < DIAGNOSTIC_SEVERITY_ERROR:
            return

        if msg.ecu_name == 'vehicle_manager':
            return

        if DEBUG_VEHICLE_MANAGER:
            self.get_logger().error(
                f'Diagnostic fault received: ecu={msg.ecu_name}, '
                f'severity={msg.severity}, description={msg.description}'
            )

        self.change_state(VehicleState_e.EMERGENCY)

    def obstacle_callback(self, msg):
        if msg.detected:
            self.handle_obstacle_detected(msg)
        else:
            self.handle_obstacle_cleared()

        if DEBUG_OBSTACLE_MSG:
            self.get_logger().info(
                f'Obstacle detected={msg.detected}, '
                f'distance={msg.distance:.2f}m, angle={msg.angle:.1f}deg'
            )

    def start_mission_callback(self, request, response):
        if self.obstacle_detected:
            response.success = False
            response.message = 'Cannot start mission while obstacle is detected'
            return response

        if self.state == VehicleState_e.READY:
            self.mission_started_ns = self.get_clock().now().nanoseconds
            self.mission_active = True
            self.change_state(VehicleState_e.MISSION)
            response.success = True
            response.message = 'Mission started'
            return response

        if self.state == VehicleState_e.MISSION:
            response.success = True
            response.message = 'Mission already running'
            return response

        if self.state == VehicleState_e.LOW_BATTERY:
            if self.mission_active:
                response.success = True
                response.message = 'Low battery mission already running'
                return response

            self.mission_started_ns = self.get_clock().now().nanoseconds
            self.mission_active = True
            self.publish_vehicle_state()
            response.success = True
            response.message = 'Low battery mission started in limp mode'
            return response

        if self.state == VehicleState_e.MRM:
            response.success = False
            response.message = 'Cannot start mission during automatic MRM'
            return response

        response.success = False
        response.message = f'Cannot start mission from {self.state.name}'
        return response

    def reset_emergency_callback(self, request, response):
        del request

        if self.state != VehicleState_e.EMERGENCY:
            response.success = False
            response.message = (
                f'Reset is only allowed from EMERGENCY, current={self.state.name}'
            )
            return response

        self.obstacle_detected = False
        self.obstacle_stop_active = False
        self.received_initial_battery_status = False
        for health in self.ecu_health.values():
            health['last_seen_ns'] = None
            health['alive'] = False

        self.change_state(VehicleState_e.INIT)
        response.success = True
        response.message = 'Emergency reset completed; reinitializing system'
        return response

    def complete_mission_callback(self, request, response):
        del request

        if self.state not in (
            VehicleState_e.MISSION,
            VehicleState_e.LOW_BATTERY,
            VehicleState_e.MRM,
        ):
            response.success = False
            response.message = f'No active mission in {self.state.name}'
            return response

        self.mission_started_ns = None
        self.mission_active = False
        if self.state == VehicleState_e.MISSION:
            self.change_state(VehicleState_e.READY)
        elif self.state == VehicleState_e.MRM:
            self.change_state(VehicleState_e.LOW_BATTERY)
        else:
            self.publish_vehicle_state()

        response.success = True
        response.message = 'Mission completed'
        return response

    def fault_callback(self, msg):
        self.change_state(VehicleState_e.EMERGENCY)

    def security_callback(self, msg):
        self.change_state(VehicleState_e.EMERGENCY)

    def security_event_callback(self, msg):
        if msg.severity < DIAGNOSTIC_SEVERITY_ERROR:
            return

        if DEBUG_VEHICLE_MANAGER:
            self.get_logger().error(
                f'Security event received: attack_type={msg.attack_type}, '
                f'severity={msg.severity}, description={msg.description}'
            )

        self.change_state(VehicleState_e.EMERGENCY)
# =============================

# =============================
# Task Implementations
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

        self.publish_heartbeat()

        if not self.check_heartbeat_timeout():
            return

        if self.state == VehicleState_e.INIT:
            if self.is_system_ready():
                self.change_state(VehicleState_e.READY)
        elif self.state == VehicleState_e.MISSION:
            if self.obstacle_stop_active:
                return

            if self.is_mission_complete():
                self.mission_started_ns = None
                self.mission_active = False
                self.change_state(VehicleState_e.READY)
        elif self.state == VehicleState_e.LOW_BATTERY:
            if (not self.obstacle_stop_active) and self.is_mission_complete():
                self.mission_started_ns = None
                self.mission_active = False
            self.check_low_battery_recovery()
        elif self.state == VehicleState_e.MRM:
            pass

        self.publish_vehicle_state()
# =============================

# =============================
# Functions
    def publish_vehicle_state(self):
        msg = VehicleState()
        msg.state = int(self.state)
        msg.mission_active = bool(self.mission_active)
        self.state_pub.publish(msg)

    def publish_heartbeat(self):
        msg = Heartbeat()
        msg.ecu_name = 'vehicle_manager'
        msg.timestamp = self.get_clock().now().nanoseconds

        self.heart_beat_publisher.publish(msg)

    def change_state(self, new_state):
        if self.state == new_state:
            return

        if new_state in (
            VehicleState_e.INIT,
            VehicleState_e.READY,
            VehicleState_e.FAULT,
            VehicleState_e.EMERGENCY,
        ):
            self.mission_started_ns = None
            self.mission_active = False
            self.resume_after_obstacle_clear = False

        if new_state != VehicleState_e.LOW_BATTERY:
            self.low_battery_recovery_started_ns = None

        if DEBUG_VEHICLE_MANAGER:
            self.get_logger().info(
                f'State Change : {self.state.name} -> {new_state.name}'
            )

        self.state = new_state
        self.publish_vehicle_state()

    def start_low_battery_mrm(self):
        if self.state in (
            VehicleState_e.FAULT,
            VehicleState_e.EMERGENCY,
            VehicleState_e.MRM,
        ):
            return

        self.mission_started_ns = None
        self.mission_active = True
        self.mrm_return_retry_count = 0
        self.change_state(VehicleState_e.MRM)
        self.send_return_home_goal()

    def send_return_home_goal(self):
        if self.mrm_return_goal_active:
            return

        if not self.return_home_client.server_is_ready():
            self.get_logger().error(
                'MRM failed: Return Home action server is unavailable'
            )
            self.change_state(VehicleState_e.EMERGENCY)
            return

        goal = ReturnHome.Goal()
        goal.start = True
        self.mrm_return_goal_active = True
        future = self.return_home_client.send_goal_async(goal)
        future.add_done_callback(self.return_home_goal_response_callback)

    def return_home_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.mrm_return_goal_active = False
            self.schedule_return_home_retry()
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.return_home_result_callback)

    def return_home_result_callback(self, future):
        self.mrm_return_goal_active = False
        result = future.result().result
        if result.success:
            return

        if self.state != VehicleState_e.MRM:
            return

        self.get_logger().error('MRM Return Home action failed')
        self.change_state(VehicleState_e.EMERGENCY)

    def schedule_return_home_retry(self):
        self.mrm_return_retry_count += 1
        if self.mrm_return_retry_count > 10:
            self.get_logger().error(
                'MRM failed: Return Home goal remained unavailable'
            )
            self.change_state(VehicleState_e.EMERGENCY)
            return

        if self.mrm_return_retry_timer is not None:
            return

        self.mrm_return_retry_timer = self.create_timer(
            0.2,
            self.retry_return_home_once
        )

    def retry_return_home_once(self):
        self.mrm_return_retry_timer.cancel()
        self.destroy_timer(self.mrm_return_retry_timer)
        self.mrm_return_retry_timer = None
        if self.state == VehicleState_e.MRM:
            self.send_return_home_goal()

    def is_system_ready(self):
        return (
            self.are_required_ecus_alive() and
            self.received_initial_battery_status
        )

    def are_required_ecus_alive(self):
        for health in self.ecu_health.values():
            if health['required'] and not health['alive']:
                return False
        return True

    def check_heartbeat_timeout(self):
        now_ns = self.get_clock().now().nanoseconds

        for ecu_name, health in self.ecu_health.items():
            last_seen_ns = health['last_seen_ns']
            if last_seen_ns is None:
                if health['required']:
                    return False
                continue

            elapsed_sec = (now_ns - last_seen_ns) / 1_000_000_000.0

            if elapsed_sec > health['timeout_sec']:
                health['alive'] = False

                if health['required']:
                    if DEBUG_HEARTBEAT_MSG:
                        self.get_logger().error(
                            f'Required ECU timeout : {ecu_name}, '
                            f'elapsed={elapsed_sec:.1f}s'
                        )
                    self.change_state(VehicleState_e.EMERGENCY)
                    return False

                if DEBUG_HEARTBEAT_MSG:
                    self.get_logger().warn(
                        f'Optional ECU timeout: {ecu_name}, '
                        f'elapsed={elapsed_sec:.1f}s'
                    )
        return True

    def is_mission_complete(self):
        if self.mission_started_ns is None:
            return False

        now_ns = self.get_clock().now().nanoseconds
        elapsed_sec = (now_ns - self.mission_started_ns) / 1_000_000_000.0
        return elapsed_sec >= self.mission_duration_sec

    def update_low_battery_recovery(self, soc):
        if soc < LOW_BATTERY_RECOVER_SOC:
            self.low_battery_recovery_started_ns = None
            return

        if self.low_battery_recovery_started_ns is None:
            self.low_battery_recovery_started_ns = (
                self.get_clock().now().nanoseconds
            )

    def check_low_battery_recovery(self):
        if self.low_battery_recovery_started_ns is None:
            return

        now_ns = self.get_clock().now().nanoseconds
        elapsed_sec = (
            now_ns - self.low_battery_recovery_started_ns
        ) / 1_000_000_000.0

        if elapsed_sec >= LOW_BATTERY_RECOVER_HOLD_SEC:
            self.change_state(VehicleState_e.INIT)

    def handle_obstacle_detected(self, msg):
        self.obstacle_detected = True

        if self.obstacle_stop_active:
            return

        if not self.mission_active:
            return

        self.obstacle_stop_active = True
        self.obstacle_pause_started_ns = self.get_clock().now().nanoseconds
        self.resume_after_obstacle_clear = (
            self.state in (
                VehicleState_e.MISSION,
                VehicleState_e.LOW_BATTERY,
                VehicleState_e.MRM,
            )
        )

        self.mission_active = False
        self.publish_vehicle_state()

        if DEBUG_VEHICLE_MANAGER:
            self.get_logger().warn(
                f'Obstacle stop: distance={msg.distance:.2f}m, '
                f'angle={msg.angle:.1f}deg'
            )

    def handle_obstacle_cleared(self):
        if not self.obstacle_detected:
            return

        was_stop_active = self.obstacle_stop_active
        self.obstacle_detected = False
        self.obstacle_stop_active = False

        if self.resume_after_obstacle_clear:
            if (
                self.mission_started_ns is not None and
                self.obstacle_pause_started_ns is not None
            ):
                pause_duration_ns = (
                    self.get_clock().now().nanoseconds -
                    self.obstacle_pause_started_ns
                )
                self.mission_started_ns += pause_duration_ns

            self.mission_active = True
            if self.mission_started_ns is None:
                self.mission_started_ns = self.get_clock().now().nanoseconds
            self.publish_vehicle_state()

        self.obstacle_pause_started_ns = None
        self.resume_after_obstacle_clear = False

        if DEBUG_VEHICLE_MANAGER and was_stop_active:
            self.get_logger().info('Obstacle cleared')
# =============================


def main(args=None):
    rclpy.init(args=args)

    node = VehicleManagerNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
