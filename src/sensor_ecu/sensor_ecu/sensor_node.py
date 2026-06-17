import rclpy
from rclpy.node import Node

from sdv_interfaces.msg import Heartbeat
from sdv_interfaces.msg import ObstacleInfo
from sdv_interfaces.msg import VehicleState
from sdv_interfaces.srv import CalibrateSensor

from sensor_ecu.sensor_driver import HwSensorDriver
from sensor_ecu.sensor_driver import SimSensorDriver

# Node Config VARs
DEBUG = False
DEBUG_OBSTACLE_MSG = False
DEBUG_VEHICLE_STATE_MSG = False
TASK_USE_1MS = False
TASK_USE_10MS = False
TASK_USE_100MS = True
TASK_USE_1000MS = True


class SensorNode(Node):

    def __init__(self):
        super().__init__('sensor_ecu')

        # =============================
        # Members
        self.declare_parameter('driver_mode', 'sim')
        self.driver_mode = self.get_parameter('driver_mode').value
        self.sensor_driver = self.create_sensor_driver(self.driver_mode)
        self.last_vehicle_state = VehicleState.INIT
        # =============================

        # =============================
        # Create Pub / Sub
        self.heart_beat_publisher = self.create_publisher(
            Heartbeat,
            '/ecu/heartbeat',
            10
        )

        self.obstacle_info_publisher = self.create_publisher(
            ObstacleInfo,
            '/ecu/obstacle/info',
            10
        )

        self.vehicle_state_sub = self.create_subscription(
            VehicleState,
            '/ecu/vehicle/status',
            self.vehicle_state_callback,
            10
        )

        self.calibrate_sensor_service = self.create_service(
            CalibrateSensor,
            '/ecu/sensor/calibrate',
            self.calibrate_sensor_callback
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
                'Sensor ECU Started'
            )

# ===================
# CALLBACKs
    def vehicle_state_callback(self, msg):
        self.last_vehicle_state = msg.state

        if DEBUG_VEHICLE_STATE_MSG:
            self.get_logger().info(
                f'Current Vehicle State = {msg.state}'
            )

    def calibrate_sensor_callback(self, request, response):
        response.success = self.sensor_driver.calibrate()

        if DEBUG:
            self.get_logger().info(
                f'Sensor calibration result = {response.success}'
            )

        return response
# =================== End of CALLBACKs

# ===================
# Task Implementation
    def Task_100ms(self):
        obstacle = self.sensor_driver.read_obstacle()
        self.publish_obstacle_info(obstacle)

    def Task_1000ms(self):
        self.publish_heartbeat()
# ===================

# ===================
# Functions
    def create_sensor_driver(self, driver_mode):
        if driver_mode == 'sim':
            return SimSensorDriver()
        if driver_mode == 'hw':
            return HwSensorDriver()

        self.get_logger().warn(
            f'Unknown sensor driver mode: {driver_mode}, fallback to sim'
        )
        return SimSensorDriver()

    def publish_heartbeat(self):
        msg = Heartbeat()
        msg.ecu_name = 'sensor_ecu'
        msg.timestamp = self.get_clock().now().nanoseconds

        self.heart_beat_publisher.publish(msg)

    def publish_obstacle_info(self, obstacle):
        msg = ObstacleInfo()
        msg.detected = bool(obstacle['detected'])
        msg.distance = float(obstacle['distance'])
        msg.angle = float(obstacle['angle'])

        self.obstacle_info_publisher.publish(msg)

        if DEBUG_OBSTACLE_MSG and msg.detected:
            self.get_logger().info(
                f'Obstacle Detected: distance={msg.distance:.2f}m, '
                f'angle={msg.angle:.1f}deg'
            )
# ===================


def main(args=None):
    rclpy.init(args=args)

    node = SensorNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
