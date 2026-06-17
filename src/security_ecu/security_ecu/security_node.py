import rclpy
from rclpy.node import Node

from sdv_interfaces.msg import BatteryStatus
from sdv_interfaces.msg import Heartbeat
from sdv_interfaces.msg import MotorStatus
from sdv_interfaces.msg import SecurityEvent
from sdv_interfaces.msg import VehicleState


# Node Config VARs
DEBUG = False
DEBUG_MONITORED_TOPICS = False
TASK_USE_1000MS = True
SECURITY_SEVERITY_ERROR = 2

BATTERY_SOC_MIN = 0.0
BATTERY_SOC_MAX = 100.0
BATTERY_VOLTAGE_MIN = 0.0
BATTERY_VOLTAGE_MAX = 80.0
BATTERY_CURRENT_ABS_MAX = 500.0
BATTERY_SOC_MAX_RATE_PER_SEC = 15.0
BATTERY_ANOMALY_LOG_COOLDOWN_SEC = 3.0

MOTOR_TARGET_LINEAR_ABS_MAX = 1.0
MOTOR_CURRENT_LINEAR_ABS_MAX = 1.2
MOTOR_TARGET_ANGULAR_ABS_MAX = 2.0
MOTOR_CURRENT_ANGULAR_ABS_MAX = 2.5
MOTOR_DISABLED_SPEED_EPSILON = 0.05
MOTOR_ANOMALY_LOG_COOLDOWN_SEC = 3.0


class SecurityNode(Node):

    def __init__(self):
        super().__init__('security_ecu')

        self.last_battery_status = None
        self.last_battery_status_ns = None
        self.last_checked_battery_status = None
        self.last_checked_battery_status_ns = None
        self.last_battery_anomalies = []
        self.last_battery_anomaly_log_ns = {}
        self.last_motor_status = None
        self.last_motor_status_ns = None
        self.last_motor_anomalies = []
        self.last_motor_anomaly_log_ns = {}
        self.last_vehicle_status = None
        self.last_monitored_topic_ns = {
            '/ecu/battery/status': None,
            '/ecu/motor/status': None,
            '/ecu/vehicle/status': None,
        }

        self.heart_beat_publisher = self.create_publisher(
            Heartbeat,
            '/ecu/heartbeat',
            10
        )

        self.security_event_publisher = self.create_publisher(
            SecurityEvent,
            '/ecu/security/event',
            10
        )

        self.battery_status_subscription = self.create_subscription(
            BatteryStatus,
            '/ecu/battery/status',
            self.battery_status_callback,
            10
        )

        self.motor_status_subscription = self.create_subscription(
            MotorStatus,
            '/ecu/motor/status',
            self.motor_status_callback,
            10
        )

        self.vehicle_status_subscription = self.create_subscription(
            VehicleState,
            '/ecu/vehicle/status',
            self.vehicle_status_callback,
            10
        )

        if TASK_USE_1000MS:
            self.timer_1000ms = self.create_timer(1.0, self.Task_1000ms)

        if DEBUG:
            self.get_logger().info('Security ECU Started')

    def battery_status_callback(self, msg):
        now_ns = self.update_monitored_topic('/ecu/battery/status')
        self.last_battery_status = msg
        self.last_battery_status_ns = now_ns

        if DEBUG_MONITORED_TOPICS:
            self.get_logger().info(
                f'Monitor battery: soc={msg.soc:.1f}, '
                f'voltage={msg.voltage:.1f}, current={msg.current:.1f}'
            )

    def motor_status_callback(self, msg):
        now_ns = self.update_monitored_topic('/ecu/motor/status')
        self.last_motor_status = msg
        self.last_motor_status_ns = now_ns

        if DEBUG_MONITORED_TOPICS:
            self.get_logger().info(
                f'Monitor motor: target_linear={msg.target_linear:.2f}, '
                f'current_linear={msg.current_linear:.2f}, '
                f'enabled={msg.enabled}'
            )

    def vehicle_status_callback(self, msg):
        self.last_vehicle_status = msg
        self.update_monitored_topic('/ecu/vehicle/status')

        if DEBUG_MONITORED_TOPICS:
            self.get_logger().info(
                f'Monitor vehicle: state={msg.state}, '
                f'mission_active={msg.mission_active}'
            )

    def Task_1000ms(self):
        self.publish_heartbeat()
        self.monitor_battery_security()
        self.monitor_motor_security()

    def update_monitored_topic(self, topic_name):
        now_ns = self.get_clock().now().nanoseconds
        self.last_monitored_topic_ns[topic_name] = now_ns
        return now_ns

    def monitor_battery_security(self):
        if self.last_battery_status is None:
            return

        if self.last_battery_status_ns is None:
            return

        anomalies = self.detect_battery_anomalies(
            self.last_battery_status,
            self.last_battery_status_ns
        )
        self.last_battery_anomalies = anomalies

        for anomaly_key, description in anomalies:
            self.report_battery_anomaly(
                anomaly_key,
                description,
                self.last_battery_status_ns
            )

        self.last_checked_battery_status = self.last_battery_status
        self.last_checked_battery_status_ns = self.last_battery_status_ns

    def detect_battery_anomalies(self, msg, msg_ns):
        anomalies = []

        if msg.soc < BATTERY_SOC_MIN or msg.soc > BATTERY_SOC_MAX:
            anomalies.append((
                'BATTERY_SOC_RANGE',
                f'Invalid battery SOC: {msg.soc:.1f}%'
            ))

        if (
            msg.voltage < BATTERY_VOLTAGE_MIN or
            msg.voltage > BATTERY_VOLTAGE_MAX
        ):
            anomalies.append((
                'BATTERY_VOLTAGE_RANGE',
                f'Invalid battery voltage: {msg.voltage:.1f}V'
            ))

        if abs(msg.current) > BATTERY_CURRENT_ABS_MAX:
            anomalies.append((
                'BATTERY_CURRENT_RANGE',
                f'Invalid battery current: {msg.current:.1f}A'
            ))

        if self.can_check_battery_soc_rate(msg, msg_ns):
            elapsed_sec = (
                msg_ns - self.last_checked_battery_status_ns
            ) / 1_000_000_000.0
            soc_delta = abs(msg.soc - self.last_checked_battery_status.soc)
            soc_rate = soc_delta / elapsed_sec

            if soc_rate > BATTERY_SOC_MAX_RATE_PER_SEC:
                anomalies.append((
                    'BATTERY_SOC_RATE',
                    f'Battery SOC changed too fast: {soc_rate:.1f}%/s'
                ))

        return anomalies

    def can_check_battery_soc_rate(self, msg, msg_ns):
        if self.last_checked_battery_status is None:
            return False

        if self.last_checked_battery_status_ns is None:
            return False

        elapsed_sec = (
            msg_ns - self.last_checked_battery_status_ns
        ) / 1_000_000_000.0
        if elapsed_sec <= 0.0:
            return False

        return (
            BATTERY_SOC_MIN <= msg.soc <= BATTERY_SOC_MAX and
            BATTERY_SOC_MIN <= self.last_checked_battery_status.soc <=
            BATTERY_SOC_MAX
        )

    def report_battery_anomaly(self, anomaly_key, description, now_ns):
        last_log_ns = self.last_battery_anomaly_log_ns.get(anomaly_key)

        if last_log_ns is not None:
            elapsed_sec = (now_ns - last_log_ns) / 1_000_000_000.0
            if elapsed_sec < BATTERY_ANOMALY_LOG_COOLDOWN_SEC:
                return

        self.last_battery_anomaly_log_ns[anomaly_key] = now_ns
        self.publish_security_event(
            anomaly_key,
            SECURITY_SEVERITY_ERROR,
            description
        )
        self.get_logger().warn(f'Battery anomaly detected: {description}')

    def monitor_motor_security(self):
        if self.last_motor_status is None:
            return

        if self.last_motor_status_ns is None:
            return

        anomalies = self.detect_motor_anomalies(self.last_motor_status)
        self.last_motor_anomalies = anomalies

        for anomaly_key, description in anomalies:
            self.report_motor_anomaly(
                anomaly_key,
                description,
                self.last_motor_status_ns
            )

    def detect_motor_anomalies(self, msg):
        anomalies = []

        if abs(msg.target_linear) > MOTOR_TARGET_LINEAR_ABS_MAX:
            anomalies.append((
                'MOTOR_TARGET_LINEAR_RANGE',
                'Invalid motor target linear speed: '
                f'{msg.target_linear:.2f}m/s'
            ))

        if abs(msg.current_linear) > MOTOR_CURRENT_LINEAR_ABS_MAX:
            anomalies.append((
                'MOTOR_CURRENT_LINEAR_RANGE',
                'Invalid motor current linear speed: '
                f'{msg.current_linear:.2f}m/s'
            ))

        if abs(msg.target_angular) > MOTOR_TARGET_ANGULAR_ABS_MAX:
            anomalies.append((
                'MOTOR_TARGET_ANGULAR_RANGE',
                'Invalid motor target angular speed: '
                f'{msg.target_angular:.2f}rad/s'
            ))

        if abs(msg.current_angular) > MOTOR_CURRENT_ANGULAR_ABS_MAX:
            anomalies.append((
                'MOTOR_CURRENT_ANGULAR_RANGE',
                'Invalid motor current angular speed: '
                f'{msg.current_angular:.2f}rad/s'
            ))

        if (
            not msg.enabled and
            (
                abs(msg.current_linear) > MOTOR_DISABLED_SPEED_EPSILON or
                abs(msg.current_angular) > MOTOR_DISABLED_SPEED_EPSILON
            )
        ):
            anomalies.append((
                'MOTOR_DISABLED_WITH_SPEED',
                'Motor reports speed while disabled'
            ))

        return anomalies

    def report_motor_anomaly(self, anomaly_key, description, now_ns):
        last_log_ns = self.last_motor_anomaly_log_ns.get(anomaly_key)

        if last_log_ns is not None:
            elapsed_sec = (now_ns - last_log_ns) / 1_000_000_000.0
            if elapsed_sec < MOTOR_ANOMALY_LOG_COOLDOWN_SEC:
                return

        self.last_motor_anomaly_log_ns[anomaly_key] = now_ns
        self.publish_security_event(
            anomaly_key,
            SECURITY_SEVERITY_ERROR,
            description
        )
        self.get_logger().warn(f'Motor anomaly detected: {description}')

    def publish_security_event(self, attack_type, severity, description):
        msg = SecurityEvent()
        msg.attack_type = attack_type
        msg.severity = int(severity)
        msg.description = description

        self.security_event_publisher.publish(msg)

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
