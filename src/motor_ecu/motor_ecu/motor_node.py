import math
import time

from motor_ecu.motor_driver import HwMotorDriver
from motor_ecu.motor_driver import SimMotorDriver
import rclpy
from rclpy.action import ActionServer
from rclpy.action import CancelResponse
from rclpy.action import GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sdv_interfaces.action import GoToTarget
from sdv_interfaces.action import ReturnHome
from sdv_interfaces.msg import Heartbeat
from sdv_interfaces.msg import MotorStatus
from sdv_interfaces.msg import VehiclePose
from sdv_interfaces.msg import VehicleState
from sdv_interfaces.srv import CompleteMission

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
        self.action_active = False
        self.action_completed_hold = False
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_yaw = 0.0
        self.motor_driver = self.create_motor_driver(self.driver_mode)
        self.action_callback_group = ReentrantCallbackGroup()
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

        self.vehicle_pose_publisher = self.create_publisher(
            VehiclePose,
            '/ecu/vehicle/pose',
            10
        )

        self.complete_mission_client = self.create_client(
            CompleteMission,
            '/ecu/vehicle/complete_mission'
        )

        self.vehicle_state_sub = self.create_subscription(
            VehicleState,
            '/ecu/vehicle/status',
            self.vehicle_state_callback,
            10
        )

        self.go_to_target_server = ActionServer(
            self,
            GoToTarget,
            '/go_to_target',
            execute_callback=self.execute_go_to_target,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.action_callback_group
        )

        self.return_home_server = ActionServer(
            self,
            ReturnHome,
            '/return_home',
            execute_callback=self.execute_return_home,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.action_callback_group
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
        if not msg.mission_active:
            self.action_completed_hold = False
        if msg.state == VehicleState.EMERGENCY:
            self.motor_driver.emergency_stop()
        elif not self.action_active and not self.action_completed_hold:
            self.apply_vehicle_state_policy()

        if DEBUG_VEHICLE_STATE_MSG:
            self.get_logger().info(
                f'Current Vehicle State = {msg.state}, '
                f'mission_active={msg.mission_active}'
            )
# =================== End of CALLBACKs

    def goal_callback(self, goal_request):
        del goal_request
        if self.action_active:
            return GoalResponse.REJECT

        if self.last_vehicle_state not in (
            VehicleState.READY,
            VehicleState.MISSION,
            VehicleState.LOW_BATTERY,
            VehicleState.MRM,
        ):
            return GoalResponse.REJECT

        self.action_completed_hold = False
        self.action_active = True
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        del goal_handle
        return CancelResponse.ACCEPT

    def execute_go_to_target(self, goal_handle):
        return self.execute_pose_action(
            goal_handle,
            GoToTarget,
            float(goal_handle.request.x),
            float(goal_handle.request.y)
        )

    def execute_return_home(self, goal_handle):
        if not goal_handle.request.start:
            self.action_active = False
            goal_handle.abort()
            result = ReturnHome.Result()
            result.success = False
            return result
        return self.execute_pose_action(
            goal_handle,
            ReturnHome,
            0.0,
            0.0
        )

    def execute_pose_action(
        self,
        goal_handle,
        action_type,
        target_x,
        target_y
    ):
        feedback = action_type.Feedback()
        initial_distance = math.hypot(
            target_x - self.pose_x,
            target_y - self.pose_y
        )
        activation_wait_started = time.monotonic()

        try:
            while True:
                if goal_handle.is_cancel_requested:
                    self.motor_driver.stop()
                    goal_handle.canceled()
                    return self.create_action_result(action_type, False)

                if self.last_vehicle_state in (
                    VehicleState.FAULT,
                    VehicleState.EMERGENCY,
                ):
                    self.motor_driver.emergency_stop()
                    goal_handle.abort()
                    return self.create_action_result(action_type, False)

                if (
                    self.last_vehicle_state == VehicleState.MRM and
                    action_type is not ReturnHome
                ):
                    self.motor_driver.stop()
                    goal_handle.abort()
                    return self.create_action_result(action_type, False)

                if self.is_action_paused():
                    self.motor_driver.stop()
                    if (
                        self.last_vehicle_state == VehicleState.READY and
                        time.monotonic() - activation_wait_started > 3.0
                    ):
                        goal_handle.abort()
                        return self.create_action_result(action_type, False)
                    time.sleep(0.1)
                    continue

                if not self.is_action_state_allowed():
                    self.motor_driver.stop()
                    goal_handle.abort()
                    return self.create_action_result(action_type, False)

                delta_x = target_x - self.pose_x
                delta_y = target_y - self.pose_y
                remaining = math.hypot(delta_x, delta_y)
                if remaining <= 0.03:
                    feedback.progress = 100.0
                    goal_handle.publish_feedback(feedback)
                    self.motor_driver.stop()
                    self.action_completed_hold = True
                    if self.complete_mission():
                        goal_handle.succeed()
                        return self.create_action_result(action_type, True)

                    goal_handle.abort()
                    return self.create_action_result(action_type, False)

                desired_heading = math.atan2(delta_y, delta_x)
                heading_error = self.normalize_angle(
                    desired_heading - self.pose_yaw
                )
                if abs(heading_error) > 0.08:
                    self.motor_driver.set_velocity(
                        0.0,
                        math.copysign(
                            min(0.8, max(0.2, abs(heading_error))),
                            heading_error
                        )
                    )
                else:
                    linear_speed = min(
                        self.get_action_linear_speed(),
                        max(0.08, remaining)
                    )
                    self.motor_driver.set_velocity(linear_speed, 0.0)

                progress = (
                    1.0 -
                    remaining / max(initial_distance, 0.001)
                ) * 100.0
                feedback.progress = float(
                    min(99.0, max(0.0, progress))
                )
                goal_handle.publish_feedback(feedback)
                time.sleep(0.1)
        finally:
            self.action_active = False

    def get_action_linear_speed(self):
        if self.last_vehicle_state in (
            VehicleState.LOW_BATTERY,
            VehicleState.MRM,
        ):
            return LOW_BATTERY_TARGET_LINEAR
        return self.mission_target_linear

    def is_action_state_allowed(self):
        return (
            self.last_vehicle_state in (
                VehicleState.MISSION,
                VehicleState.LOW_BATTERY,
                VehicleState.MRM,
            ) and
            self.last_mission_active
        )

    def is_action_paused(self):
        return (
            self.last_vehicle_state in (
                VehicleState.READY,
                VehicleState.MISSION,
                VehicleState.LOW_BATTERY,
                VehicleState.MRM,
            ) and
            not self.last_mission_active
        )

    def create_action_result(self, action_type, success):
        result = action_type.Result()
        result.success = bool(success)
        return result

    def complete_mission(self):
        if not self.complete_mission_client.service_is_ready():
            self.get_logger().warn('Complete Mission service is not available')
            return False

        future = self.complete_mission_client.call_async(
            CompleteMission.Request()
        )
        deadline = time.monotonic() + 2.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)

        if not future.done():
            self.get_logger().error('Complete Mission service timed out')
            return False

        response = future.result()
        if response is None or not response.success:
            message = response.message if response is not None else 'no response'
            self.get_logger().error(f'Mission completion failed: {message}')
            return False
        return True

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

# ===================
# Task Implementation
    def Task_100ms(self):
        if not self.action_active and not self.action_completed_hold:
            self.apply_vehicle_state_policy()
        self.motor_driver.update(0.1)
        self.update_pose(0.1)
        self.publish_motor_status()
        self.publish_vehicle_pose()

    def Task_1000ms(self):
        self.publish_heartbeat()
# ===================

# ===================
# Functions
    def create_motor_driver(self, driver_mode):
        if driver_mode == 'sim':
            return SimMotorDriver()
        if driver_mode == 'hw':
            return HwMotorDriver()

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
            self.last_vehicle_state in (
                VehicleState.LOW_BATTERY,
                VehicleState.MRM,
            ) and
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

    def update_pose(self, dt_sec):
        status = self.motor_driver.get_status()
        current_linear = float(status['current_linear'])
        current_angular = float(status['current_angular'])
        self.pose_yaw = self.normalize_angle(
            self.pose_yaw + current_angular * dt_sec
        )
        self.pose_x += current_linear * math.cos(self.pose_yaw) * dt_sec
        self.pose_y += current_linear * math.sin(self.pose_yaw) * dt_sec

    def publish_vehicle_pose(self):
        msg = VehiclePose()
        msg.x = float(self.pose_x)
        msg.y = float(self.pose_y)
        msg.yaw = float(self.pose_yaw)
        self.vehicle_pose_publisher.publish(msg)
# ===================


def main(args=None):
    rclpy.init(args=args)

    node = MotorNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)

    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    executor.shutdown()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
