from enum import IntEnum

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sdv_interfaces.msg import BatteryStatus
from sdv_interfaces.msg import DiagnosticEvent
from sdv_interfaces.msg import Heartbeat
from sdv_interfaces.msg import VehicleState
from sdv_interfaces.srv import ClearFault
from sdv_interfaces.srv import GetFaultInfo

# Node Config VARs
DEBUG = True
TASK_USE_1MS = False
TASK_USE_10MS = False
TASK_USE_100MS = False
TASK_USE_1000MS = True


class DiagnosticSeverity_e(IntEnum):
    INFO = 0
    WARN = 1
    ERROR = 2
    CRITICAL = 3


class DiagnosticsNode(Node):

    def __init__(self):
        super().__init__('diagnostics_ecu')

        # =============================
        # Members
        self.low_battery_threshold = 20.0
        self.event_cooldown_sec = 3.0
        self.last_event_publish_ns = {}
        self.last_vehicle_state = None
        self.active_faults = {}

        self.ecu_health = {
            'vehicle_manager': {
                'required': True,
                'timeout_sec': 3.0,
                'last_seen_ns': None,
                'alive': False,
            },
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

        # =============================
        # Create Pub / Sub
        self.heart_beat_publisher = self.create_publisher(
            Heartbeat,
            '/ecu/heartbeat',
            10
        )

        self.diagnostic_event_pub = self.create_publisher(
            DiagnosticEvent,
            '/ecu/diagnostics/event',
            10
        )

        self.battery_status_sub = self.create_subscription(
            BatteryStatus,
            '/ecu/battery/status',
            self.battery_status_callback,
            10
        )

        self.heartbeat_sub = self.create_subscription(
            Heartbeat,
            '/ecu/heartbeat',
            self.heart_beat_callback,
            10
        )

        self.vehicle_state_sub = self.create_subscription(
            VehicleState,
            '/ecu/vehicle/status',
            self.vehicle_state_callback,
            10
        )

        self.get_fault_info_service = self.create_service(
            GetFaultInfo,
            '/ecu/diagnostics/get_fault_info',
            self.get_fault_info_callback
        )

        self.clear_fault_service = self.create_service(
            ClearFault,
            '/ecu/diagnostics/clear_fault',
            self.clear_fault_callback
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
                'Diagnostics ECU Started'
            )

# ===================
# CALLBACKs
    def battery_status_callback(self, msg):
        if msg.soc <= self.low_battery_threshold:
            self.publish_diagnostic_event(
                'battery_ecu',
                DiagnosticSeverity_e.WARN,
                f'Low battery SOC detected: {msg.soc:.1f}%'
            )

    def heart_beat_callback(self, msg):
        ecu_name = msg.ecu_name

        if ecu_name not in self.ecu_health:
            self.publish_diagnostic_event(
                ecu_name,
                DiagnosticSeverity_e.WARN,
                f'Unknown ECU heartbeat received: {ecu_name}'
            )
            return

        self.ecu_health[ecu_name]['last_seen_ns'] = (
            self.get_clock().now().nanoseconds
        )
        self.ecu_health[ecu_name]['alive'] = True

    def vehicle_state_callback(self, msg):
        if self.last_vehicle_state == msg.state:
            return

        self.last_vehicle_state = msg.state

        if msg.state == VehicleState.FAULT:
            self.publish_diagnostic_event(
                'vehicle_manager',
                DiagnosticSeverity_e.ERROR,
                'Vehicle state changed to FAULT'
            )
        elif msg.state == VehicleState.EMERGENCY:
            self.publish_diagnostic_event(
                'vehicle_manager',
                DiagnosticSeverity_e.CRITICAL,
                'Vehicle state changed to EMERGENCY'
            )

    def get_fault_info_callback(self, request, response):
        del request
        response.has_fault = bool(self.active_faults)
        response.description = '; '.join(
            f'{name}: {description}'
            for name, description in sorted(self.active_faults.items())
        )
        return response

    def clear_fault_callback(self, request, response):
        fault_name = request.fault_name.strip()
        if not fault_name or fault_name == 'all':
            self.active_faults.clear()
            response.success = True
            return response

        response.success = self.active_faults.pop(fault_name, None) is not None
        return response
# =================== End of CALLBACKs

# ===================
# Task Implementation
    def Task_1000ms(self):
        self.publish_heartbeat()
        self.check_heartbeat_timeout()
# ===================

# ===================
# Functions
    def publish_heartbeat(self):
        msg = Heartbeat()
        msg.ecu_name = 'diagnostics_ecu'
        msg.timestamp = self.get_clock().now().nanoseconds

        self.heart_beat_publisher.publish(msg)

    def publish_diagnostic_event(self, ecu_name, severity, description):
        event_key = f'{ecu_name}:{int(severity)}:{description}'

        if not self.can_publish_event(event_key):
            return

        msg = DiagnosticEvent()
        msg.ecu_name = ecu_name
        msg.severity = int(severity)
        msg.description = description

        self.diagnostic_event_pub.publish(msg)
        self.last_event_publish_ns[event_key] = self.get_clock().now().nanoseconds
        if int(severity) >= int(DiagnosticSeverity_e.ERROR):
            self.active_faults[ecu_name] = description

        if DEBUG:
            self.get_logger().warn(
                f'Diagnostic Event: ecu={msg.ecu_name}, '
                f'severity={msg.severity}, description={msg.description}'
            )

    def can_publish_event(self, event_key):
        now_ns = self.get_clock().now().nanoseconds
        last_publish_ns = self.last_event_publish_ns.get(event_key)

        if last_publish_ns is None:
            return True

        elapsed_sec = (now_ns - last_publish_ns) / 1_000_000_000.0
        return elapsed_sec >= self.event_cooldown_sec

    def check_heartbeat_timeout(self):
        now_ns = self.get_clock().now().nanoseconds

        for ecu_name, health in self.ecu_health.items():
            last_seen_ns = health['last_seen_ns']

            if last_seen_ns is None:
                if health['required']:
                    self.publish_diagnostic_event(
                        ecu_name,
                        DiagnosticSeverity_e.WARN,
                        f'Required ECU heartbeat not received: {ecu_name}'
                    )
                continue

            elapsed_sec = (now_ns - last_seen_ns) / 1_000_000_000.0

            if elapsed_sec > health['timeout_sec']:
                if not health['alive']:
                    continue

                health['alive'] = False

                if health['required']:
                    self.publish_diagnostic_event(
                        ecu_name,
                        DiagnosticSeverity_e.ERROR,
                        f'Required ECU timeout: {ecu_name}, elapsed={elapsed_sec:.1f}s'
                    )
                else:
                    self.publish_diagnostic_event(
                        ecu_name,
                        DiagnosticSeverity_e.WARN,
                        f'Optional ECU timeout: {ecu_name}, '
                        f'elapsed={elapsed_sec:.1f}s'
                    )
# ===================


def main(args=None):
    rclpy.init(args=args)

    node = DiagnosticsNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
