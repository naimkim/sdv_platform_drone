from datetime import datetime
import math
import sys
import threading
import time

from PyQt5.QtCore import pyqtSignal, QObject, QPointF, QRectF, Qt, QTimer
from PyQt5.QtGui import QColor, QFont, QPainter, QPen
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QListWidget,
    QMainWindow,
    QPushButton,
    QScrollArea,
    QSlider,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sdv_interfaces.msg import BatteryStatus
from sdv_interfaces.msg import DiagnosticEvent
from sdv_interfaces.msg import Heartbeat
from sdv_interfaces.msg import MotorStatus
from sdv_interfaces.msg import ObstacleInfo
from sdv_interfaces.msg import VehicleState
from sdv_interfaces.srv import StartMission

MAIN_WINDOW_WIDTH = 1100
MAIN_WINDOW_HEIGHT = 760
SIMULATION_UPDATE_INTERVAL_MS = 50
WORLD_BOUNDARY_M = 4.0
GUI_FONT_FAMILY = 'DejaVu Sans'

ECU_NAMES = (
    'battery_ecu',
    'sensor_ecu',
    'motor_ecu',
    'security_ecu',
)

VEHICLE_STATE_NAMES = {
    VehicleState.INIT: 'INIT',
    VehicleState.READY: 'READY',
    VehicleState.MISSION: 'MISSION',
    VehicleState.LOW_BATTERY: 'LOW_BATTERY',
    VehicleState.FAULT: 'FAULT',
    VehicleState.EMERGENCY: 'EMERGENCY',
}

DIAGNOSTIC_SEVERITY_NAMES = {
    0: 'INFO',
    1: 'WARN',
    2: 'ERROR',
    3: 'CRITICAL',
}

STATE_COLORS = {
    VehicleState.INIT: QColor(120, 120, 120),
    VehicleState.READY: QColor(36, 128, 74),
    VehicleState.MISSION: QColor(32, 94, 166),
    VehicleState.LOW_BATTERY: QColor(186, 126, 18),
    VehicleState.FAULT: QColor(184, 54, 54),
    VehicleState.EMERGENCY: QColor(150, 28, 28),
}


class GuiSignals(QObject):
    node_list_received = pyqtSignal(object)
    vehicle_state_received = pyqtSignal(int)
    battery_status_received = pyqtSignal(float, float, float)
    obstacle_info_received = pyqtSignal(bool, float, float)
    motor_status_received = pyqtSignal(float, float, float, float, bool)
    heartbeat_received = pyqtSignal(str, str, float)
    diagnostic_event_received = pyqtSignal(str, int, str, str)
    service_result_received = pyqtSignal(str, bool, str)


class SdvTestGuiRosNode(Node):

    def __init__(self, gui_signals):
        super().__init__('sdv_test_gui')

        self.gui_signals = gui_signals

        self.battery_pub = self.create_publisher(
            BatteryStatus,
            '/ecu/battery/status',
            10
        )

        self.heartbeat_pub = self.create_publisher(
            Heartbeat,
            '/ecu/heartbeat',
            10
        )

        self.start_mission_client = self.create_client(
            StartMission,
            '/ecu/vehicle/start_mission'
        )

        self.vehicle_state_sub = self.create_subscription(
            VehicleState,
            '/ecu/vehicle/status',
            self.vehicle_state_callback,
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
            self.heartbeat_callback,
            10
        )

        self.obstacle_info_sub = self.create_subscription(
            ObstacleInfo,
            '/ecu/obstacle/info',
            self.obstacle_info_callback,
            10
        )

        self.motor_status_sub = self.create_subscription(
            MotorStatus,
            '/ecu/motor/status',
            self.motor_status_callback,
            10
        )

        self.diagnostic_event_sub = self.create_subscription(
            DiagnosticEvent,
            '/ecu/diagnostics/event',
            self.diagnostic_event_callback,
            10
        )

        self.node_monitor_timer = self.create_timer(
            1.0,
            self.publish_node_list_to_gui
        )

    def vehicle_state_callback(self, msg):
        self.gui_signals.vehicle_state_received.emit(int(msg.state))

    def battery_status_callback(self, msg):
        self.gui_signals.battery_status_received.emit(
            float(msg.soc),
            float(msg.voltage),
            float(msg.current)
        )

    def heartbeat_callback(self, msg):
        self.gui_signals.heartbeat_received.emit(
            msg.ecu_name,
            format_timestamp_ns(msg.timestamp),
            time.monotonic()
        )

    def obstacle_info_callback(self, msg):
        self.gui_signals.obstacle_info_received.emit(
            bool(msg.detected),
            float(msg.distance),
            float(msg.angle)
        )

    def motor_status_callback(self, msg):
        self.gui_signals.motor_status_received.emit(
            float(msg.target_linear),
            float(msg.current_linear),
            float(msg.target_angular),
            float(msg.current_angular),
            bool(msg.enabled)
        )

    def diagnostic_event_callback(self, msg):
        self.gui_signals.diagnostic_event_received.emit(
            msg.ecu_name,
            int(msg.severity),
            msg.description,
            format_wall_time()
        )

    def publish_node_list_to_gui(self):
        node_names = sorted(self.get_node_names())
        self.gui_signals.node_list_received.emit(node_names)

    def publish_battery(self, soc, voltage, current):
        msg = BatteryStatus()
        msg.soc = float(soc)
        msg.voltage = float(voltage)
        msg.current = float(current)
        self.battery_pub.publish(msg)

    def publish_heartbeat(self, ecu_name):
        msg = Heartbeat()
        msg.ecu_name = ecu_name
        msg.timestamp = self.get_clock().now().nanoseconds
        self.heartbeat_pub.publish(msg)

    def request_start_mission(self):
        if not self.start_mission_client.service_is_ready():
            self.gui_signals.service_result_received.emit(
                'Start Mission',
                False,
                'Service not available'
            )
            return

        request = StartMission.Request()
        future = self.start_mission_client.call_async(request)
        future.add_done_callback(self.start_mission_response_callback)

    def start_mission_response_callback(self, future):
        try:
            response = future.result()
            self.gui_signals.service_result_received.emit(
                'Start Mission',
                bool(response.success),
                response.message
            )
        except Exception as exc:
            self.gui_signals.service_result_received.emit(
                'Start Mission',
                False,
                str(exc)
            )


class SimulationView(QWidget):

    def __init__(self):
        super().__init__()

        self.setMinimumHeight(280)

        self.x_m = 0.0
        self.y_m = 0.0
        self.yaw_rad = -math.pi / 2.0
        self.path_points = []

        self.current_linear = 0.0
        self.current_angular = 0.0
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.motor_enabled = False

        self.obstacle_detected = False
        self.obstacle_distance = 3.0
        self.obstacle_angle_deg = 0.0

        self.vehicle_state = VehicleState.INIT
        self.last_update_monotonic = time.monotonic()

        self.update_timer = QTimer(self)
        self.update_timer.setInterval(SIMULATION_UPDATE_INTERVAL_MS)
        self.update_timer.timeout.connect(self.step_simulation)
        self.update_timer.start()

    def set_vehicle_state(self, state):
        self.vehicle_state = state
        self.update()

    def set_obstacle(self, detected, distance, angle):
        self.obstacle_detected = detected
        self.obstacle_distance = distance
        self.obstacle_angle_deg = angle
        self.update()

    def set_motor_status(
        self,
        target_linear,
        current_linear,
        target_angular,
        current_angular,
        enabled
    ):
        self.target_linear = target_linear
        self.current_linear = current_linear
        self.target_angular = target_angular
        self.current_angular = current_angular
        self.motor_enabled = enabled

    def reset_pose(self):
        self.x_m = 0.0
        self.y_m = 0.0
        self.yaw_rad = -math.pi / 2.0
        self.path_points.clear()
        self.update()

    def step_simulation(self):
        now = time.monotonic()
        dt_sec = min(0.2, now - self.last_update_monotonic)
        self.last_update_monotonic = now

        if self.motor_enabled:
            self.yaw_rad += self.current_angular * dt_sec
            self.x_m += self.current_linear * math.cos(self.yaw_rad) * dt_sec
            self.y_m += self.current_linear * math.sin(self.yaw_rad) * dt_sec

            if not self.path_points or distance_2d(
                self.path_points[-1][0],
                self.path_points[-1][1],
                self.x_m,
                self.y_m
            ) > 0.05:
                self.path_points.append((self.x_m, self.y_m))
                self.path_points = self.path_points[-120:]

        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        rect = self.rect()
        painter.fillRect(rect, QColor(246, 248, 250))

        margin = 18
        world_rect = rect.adjusted(margin, margin, -margin, -margin)
        scale = min(world_rect.width(), world_rect.height()) / (
            WORLD_BOUNDARY_M * 2.0
        )
        vehicle_screen_center = world_rect.center()

        def to_screen(x_m, y_m):
            return QPointF(
                vehicle_screen_center.x() + (x_m - self.x_m) * scale,
                vehicle_screen_center.y() + (y_m - self.y_m) * scale
            )

        self.draw_grid(painter, world_rect, vehicle_screen_center, scale)
        self.draw_path(painter, to_screen)
        self.draw_sensor(painter, to_screen, scale)
        self.draw_vehicle(painter, to_screen)
        self.draw_obstacle(painter, to_screen)
        self.draw_overlay(painter, rect)

    def draw_grid(self, painter, world_rect, vehicle_screen_center, scale):
        painter.setPen(QPen(QColor(220, 226, 232), 1))

        start_x_m = math.floor(self.x_m - WORLD_BOUNDARY_M)
        end_x_m = math.ceil(self.x_m + WORLD_BOUNDARY_M)
        start_y_m = math.floor(self.y_m - WORLD_BOUNDARY_M)
        end_y_m = math.ceil(self.y_m + WORLD_BOUNDARY_M)

        for x_m in range(start_x_m, end_x_m + 1):
            screen_x = vehicle_screen_center.x() + (x_m - self.x_m) * scale
            painter.drawLine(
                QPointF(screen_x, world_rect.top()),
                QPointF(screen_x, world_rect.bottom())
            )

        for y_m in range(start_y_m, end_y_m + 1):
            screen_y = vehicle_screen_center.y() + (y_m - self.y_m) * scale
            painter.drawLine(
                QPointF(world_rect.left(), screen_y),
                QPointF(world_rect.right(), screen_y)
            )

        painter.setPen(QPen(QColor(135, 144, 153), 2))
        painter.drawLine(
            QPointF(vehicle_screen_center.x(), world_rect.top()),
            QPointF(vehicle_screen_center.x(), world_rect.bottom())
        )
        painter.drawLine(
            QPointF(world_rect.left(), vehicle_screen_center.y()),
            QPointF(world_rect.right(), vehicle_screen_center.y())
        )

    def draw_path(self, painter, to_screen):
        if len(self.path_points) < 2:
            return

        painter.setPen(QPen(QColor(84, 138, 198), 2))
        for index in range(1, len(self.path_points)):
            prev = self.path_points[index - 1]
            current = self.path_points[index]
            painter.drawLine(to_screen(prev[0], prev[1]), to_screen(current[0], current[1]))

    def draw_sensor(self, painter, to_screen, scale):
        origin = to_screen(self.x_m, self.y_m)
        radius = 2.2 * scale
        start_deg = -math.degrees(self.yaw_rad) - 30.0

        painter.setPen(QPen(QColor(45, 126, 171), 1))
        painter.setBrush(QColor(45, 126, 171, 36))
        painter.drawPie(
            QRectF(origin.x() - radius, origin.y() - radius, radius * 2, radius * 2),
            int(start_deg * 16),
            int(60.0 * 16)
        )

    def draw_obstacle(self, painter, to_screen):
        if not self.obstacle_detected:
            return

        obstacle_yaw = self.yaw_rad + math.radians(self.obstacle_angle_deg)
        obstacle_x = self.x_m + self.obstacle_distance * math.cos(obstacle_yaw)
        obstacle_y = self.y_m + self.obstacle_distance * math.sin(obstacle_yaw)
        obstacle_point = to_screen(obstacle_x, obstacle_y)

        painter.setPen(QPen(QColor(120, 24, 24), 3))
        painter.setBrush(QColor(216, 65, 65))
        painter.drawEllipse(obstacle_point, 10, 10)
        painter.setBrush(QColor(255, 240, 120))
        painter.drawEllipse(obstacle_point, 4, 4)
        painter.drawLine(to_screen(self.x_m, self.y_m), obstacle_point)

    def draw_vehicle(self, painter, to_screen):
        center = to_screen(self.x_m, self.y_m)

        state_color = STATE_COLORS.get(self.vehicle_state, QColor(120, 120, 120))

        painter.save()
        painter.translate(center)
        painter.rotate(math.degrees(self.yaw_rad) + 90.0)

        painter.setPen(QPen(QColor(30, 34, 38), 2))
        painter.setBrush(state_color)
        painter.drawRoundedRect(QRectF(-13.0, -23.0, 26.0, 46.0), 4.0, 4.0)

        painter.setPen(QPen(QColor(42, 48, 55), 1))
        painter.setBrush(QColor(188, 217, 235))
        painter.drawRoundedRect(QRectF(-8.0, -13.0, 16.0, 16.0), 3.0, 3.0)

        painter.setBrush(QColor(224, 232, 238))
        painter.drawRoundedRect(QRectF(-7.0, -20.0, 14.0, 6.0), 2.0, 2.0)

        painter.setBrush(QColor(34, 37, 41))
        painter.drawRoundedRect(QRectF(-18.0, -17.0, 5.0, 11.0), 2.0, 2.0)
        painter.drawRoundedRect(QRectF(13.0, -17.0, 5.0, 11.0), 2.0, 2.0)
        painter.drawRoundedRect(QRectF(-18.0, 7.0, 5.0, 11.0), 2.0, 2.0)
        painter.drawRoundedRect(QRectF(13.0, 7.0, 5.0, 11.0), 2.0, 2.0)

        painter.setPen(QPen(QColor(30, 34, 38), 2))
        painter.drawLine(QPointF(0.0, -23.0), QPointF(0.0, -32.0))

        painter.restore()

    def draw_overlay(self, painter, rect):
        painter.setPen(QColor(35, 40, 45))
        painter.setFont(QFont(GUI_FONT_FAMILY, 9))

        state_name = VEHICLE_STATE_NAMES.get(self.vehicle_state, 'UNKNOWN')
        text = (
            f'2D Simulation | state={state_name} | '
            f'v={self.current_linear:.2f} m/s | w={self.current_angular:.2f} rad/s'
        )
        painter.drawText(rect.adjusted(12, 8, -12, -8), Qt.AlignLeft | Qt.AlignTop, text)


class DashboardWidget(QWidget):

    def __init__(self):
        super().__init__()

        self.setMinimumHeight(150)

        self.vehicle_state = VehicleState.INIT
        self.soc = 0.0
        self.voltage = 0.0
        self.current = 0.0
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.motor_enabled = False
        self.obstacle_detected = False
        self.obstacle_distance = 0.0

    def set_vehicle_state(self, state):
        self.vehicle_state = state
        self.update()

    def set_battery_status(self, soc, voltage, current):
        self.soc = soc
        self.voltage = voltage
        self.current = current
        self.update()

    def set_obstacle(self, detected, distance):
        self.obstacle_detected = detected
        self.obstacle_distance = distance
        self.update()

    def set_motor_status(self, current_linear, current_angular, enabled):
        self.current_linear = current_linear
        self.current_angular = current_angular
        self.motor_enabled = enabled
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor(250, 251, 252))

        content = self.rect().adjusted(14, 12, -14, -12)
        column_width = content.width() / 3.0
        self.draw_state_lamps(
            painter,
            QRectF(content.left(), content.top(), column_width, content.height())
        )
        self.draw_motion_gauges(
            painter,
            QRectF(content.left() + column_width, content.top(), column_width, content.height())
        )
        self.draw_battery_obstacle(
            painter,
            QRectF(
                content.left() + column_width * 2.0,
                content.top(),
                column_width,
                content.height()
            )
        )

    def draw_state_lamps(self, painter, rect):
        state_name = VEHICLE_STATE_NAMES.get(self.vehicle_state, 'UNKNOWN')
        state_color = STATE_COLORS.get(self.vehicle_state, QColor(120, 120, 120))

        self.draw_label(painter, rect, 'Vehicle')
        painter.setPen(QPen(QColor(48, 54, 61), 1))
        painter.setBrush(state_color)
        painter.drawEllipse(QPointF(rect.left() + 20, rect.top() + 48), 13, 13)
        painter.setPen(QColor(30, 34, 38))
        painter.setFont(QFont(GUI_FONT_FAMILY, 13, QFont.Bold))
        painter.drawText(
            QRectF(rect.left() + 44, rect.top() + 32, rect.width() - 48, 36),
            Qt.AlignVCenter | Qt.AlignLeft,
            state_name
        )

        motor_color = QColor(36, 128, 74) if self.motor_enabled else QColor(145, 150, 156)
        painter.setBrush(motor_color)
        painter.drawEllipse(QPointF(rect.left() + 20, rect.top() + 88), 9, 9)
        painter.setFont(QFont(GUI_FONT_FAMILY, 10))
        painter.drawText(
            QRectF(rect.left() + 40, rect.top() + 76, rect.width() - 44, 28),
            Qt.AlignVCenter | Qt.AlignLeft,
            'Motor Enabled' if self.motor_enabled else 'Motor Disabled'
        )

    def draw_motion_gauges(self, painter, rect):
        self.draw_label(painter, rect, 'Motion')
        self.draw_bar(
            painter,
            rect.left() + 12,
            rect.top() + 42,
            rect.width() - 24,
            'Linear',
            self.current_linear,
            -1.5,
            1.5,
            'm/s'
        )
        self.draw_bar(
            painter,
            rect.left() + 12,
            rect.top() + 90,
            rect.width() - 24,
            'Angular',
            self.current_angular,
            -2.0,
            2.0,
            'rad/s'
        )

    def draw_battery_obstacle(self, painter, rect):
        self.draw_label(painter, rect, 'Power / Sensor')
        self.draw_bar(
            painter,
            rect.left() + 12,
            rect.top() + 42,
            rect.width() - 24,
            'SOC',
            self.soc,
            0.0,
            100.0,
            '%'
        )

        painter.setPen(QColor(30, 34, 38))
        painter.setFont(QFont(GUI_FONT_FAMILY, 10))
        battery_text = f'{self.voltage:.1f} V  {self.current:.1f} A'
        painter.drawText(
            QRectF(rect.left() + 12, rect.top() + 74, rect.width() - 24, 24),
            Qt.AlignLeft | Qt.AlignVCenter,
            battery_text
        )

        obstacle_color = QColor(216, 65, 65) if self.obstacle_detected else QColor(36, 128, 74)
        painter.setBrush(obstacle_color)
        painter.setPen(QPen(QColor(48, 54, 61), 1))
        painter.drawEllipse(QPointF(rect.left() + 22, rect.top() + 114), 9, 9)
        painter.setPen(QColor(30, 34, 38))
        sensor_text = (
            f'Obstacle {self.obstacle_distance:.2f} m'
            if self.obstacle_detected
            else 'Path Clear'
        )
        painter.drawText(
            QRectF(rect.left() + 42, rect.top() + 101, rect.width() - 48, 28),
            Qt.AlignLeft | Qt.AlignVCenter,
            sensor_text
        )

    def draw_bar(self, painter, left, top, width, label, value, minimum, maximum, unit):
        label_rect = QRectF(left, top - 20, width, 18)
        painter.setPen(QColor(30, 34, 38))
        painter.setFont(QFont(GUI_FONT_FAMILY, 9))
        painter.drawText(
            label_rect,
            Qt.AlignLeft | Qt.AlignVCenter,
            f'{label}: {value:.2f} {unit}'
        )

        bar_rect = QRectF(left, top, width, 14)
        painter.setPen(QPen(QColor(185, 194, 203), 1))
        painter.setBrush(QColor(229, 234, 239))
        painter.drawRoundedRect(bar_rect, 3, 3)

        normalized = 0.0
        if maximum > minimum:
            normalized = (clamp(value, minimum, maximum) - minimum) / (maximum - minimum)

        fill_width = bar_rect.width() * normalized
        fill_color = QColor(44, 119, 190)
        if label == 'SOC' and value < 25.0:
            fill_color = QColor(198, 73, 57)

        painter.setPen(Qt.NoPen)
        painter.setBrush(fill_color)
        painter.drawRoundedRect(
            QRectF(bar_rect.left(), bar_rect.top(), fill_width, bar_rect.height()),
            3,
            3
        )

        if minimum < 0.0 < maximum:
            zero_x = bar_rect.left() + bar_rect.width() * ((0.0 - minimum) / (maximum - minimum))
            painter.setPen(QPen(QColor(72, 80, 88), 1))
            painter.drawLine(
                QPointF(zero_x, bar_rect.top() - 2),
                QPointF(zero_x, bar_rect.bottom() + 2)
            )

    def draw_label(self, painter, rect, text):
        painter.setPen(QColor(85, 92, 100))
        painter.setFont(QFont(GUI_FONT_FAMILY, 9, QFont.Bold))
        painter.drawText(rect.adjusted(8, 0, -8, 0), Qt.AlignLeft | Qt.AlignTop, text)


class MainWindow(QMainWindow):

    def __init__(self, ros_node, gui_signals):
        super().__init__()

        self.ros_node = ros_node
        self.gui_signals = gui_signals
        self.last_heartbeat_times = {}

        self.setWindowTitle('SDV Test GUI')
        self.resize(MAIN_WINDOW_WIDTH, MAIN_WINDOW_HEIGHT)

        self.create_monitor_widgets()
        self.create_simulator_widgets()
        self.connect_signals()

        self.heartbeat_publish_timer = QTimer(self)
        self.heartbeat_publish_timer.setInterval(1000)
        self.heartbeat_publish_timer.timeout.connect(self.publish_enabled_heartbeats)
        self.heartbeat_publish_timer.start()

        self.monitor_refresh_timer = QTimer(self)
        self.monitor_refresh_timer.setInterval(500)
        self.monitor_refresh_timer.timeout.connect(self.refresh_heartbeat_age)
        self.monitor_refresh_timer.start()

        self.setCentralWidget(self.build_layout())

    def create_monitor_widgets(self):
        self.simulation_view = SimulationView()
        self.dashboard_widget = DashboardWidget()

        self.node_list = QListWidget()

        self.state_label = QLabel('NO DATA')
        self.state_label.setStyleSheet('font-size: 24px; font-weight: 600;')

        self.battery_soc_label = QLabel('-')
        self.battery_voltage_label = QLabel('-')
        self.battery_current_label = QLabel('-')

        self.obstacle_detected_label = QLabel('-')
        self.obstacle_distance_label = QLabel('-')
        self.obstacle_angle_label = QLabel('-')

        self.motor_enabled_label = QLabel('-')
        self.motor_target_linear_label = QLabel('-')
        self.motor_current_linear_label = QLabel('-')
        self.motor_target_angular_label = QLabel('-')
        self.motor_current_angular_label = QLabel('-')

        self.service_result_label = QLabel('-')

        self.heartbeat_table = QTableWidget(len(ECU_NAMES), 3)
        self.heartbeat_table.setHorizontalHeaderLabels([
            'ECU',
            'Last RX',
            'Sent Time',
        ])
        self.heartbeat_table.verticalHeader().setVisible(False)
        self.heartbeat_table.horizontalHeader().setSectionResizeMode(
            QHeaderView.Stretch
        )
        self.heartbeat_table.setEditTriggers(QTableWidget.NoEditTriggers)

        self.heartbeat_rows = {}
        for row, ecu_name in enumerate(ECU_NAMES):
            self.heartbeat_table.setItem(row, 0, QTableWidgetItem(ecu_name))
            self.heartbeat_table.setItem(row, 1, QTableWidgetItem('never'))
            self.heartbeat_table.setItem(row, 2, QTableWidgetItem('-'))
            self.heartbeat_rows[ecu_name] = row

        self.diagnostic_event_table = QTableWidget(0, 4)
        self.diagnostic_event_table.setHorizontalHeaderLabels([
            'RX',
            'ECU',
            'Severity',
            'Description',
        ])
        self.diagnostic_event_table.verticalHeader().setVisible(False)
        self.diagnostic_event_table.horizontalHeader().setSectionResizeMode(
            QHeaderView.Stretch
        )
        self.diagnostic_event_table.setEditTriggers(QTableWidget.NoEditTriggers)

    def create_simulator_widgets(self):
        self.soc_slider = QSlider(Qt.Horizontal)
        self.soc_slider.setRange(0, 100)
        self.soc_slider.setValue(80)

        self.soc_value_label = QLabel('80 %')

        self.voltage_spin = QDoubleSpinBox()
        self.voltage_spin.setRange(0.0, 1000.0)
        self.voltage_spin.setValue(48.0)
        self.voltage_spin.setSuffix(' V')

        self.current_spin = QDoubleSpinBox()
        self.current_spin.setRange(-1000.0, 1000.0)
        self.current_spin.setValue(0.0)
        self.current_spin.setSuffix(' A')

        self.publish_battery_button = QPushButton('Publish Battery')
        self.start_mission_button = QPushButton('Start Mission')

        self.heartbeat_checks = {}
        for ecu_name in ECU_NAMES:
            checkbox = QCheckBox(ecu_name)
            checkbox.setChecked(False)
            self.heartbeat_checks[ecu_name] = checkbox

    def connect_signals(self):
        self.gui_signals.node_list_received.connect(self.update_node_list)
        self.gui_signals.vehicle_state_received.connect(self.update_vehicle_state)
        self.gui_signals.battery_status_received.connect(self.update_battery_status)
        self.gui_signals.obstacle_info_received.connect(self.update_obstacle_info)
        self.gui_signals.motor_status_received.connect(self.update_motor_status)
        self.gui_signals.heartbeat_received.connect(self.update_heartbeat_status)
        self.gui_signals.diagnostic_event_received.connect(
            self.add_diagnostic_event
        )
        self.gui_signals.service_result_received.connect(self.update_service_result)

        self.soc_slider.valueChanged.connect(self.update_soc_label)
        self.publish_battery_button.clicked.connect(self.publish_battery)
        self.start_mission_button.clicked.connect(self.request_start_mission)

    def build_layout(self):
        root = QWidget()
        root_layout = QHBoxLayout(root)

        monitor_panel = QWidget()
        monitor_layout = QVBoxLayout(monitor_panel)
        monitor_layout.addWidget(self.build_virtual_render_group())
        monitor_layout.addWidget(self.build_dashboard_group())
        monitor_layout.addWidget(self.build_node_monitor_group())
        monitor_layout.addWidget(self.build_vehicle_state_group())
        monitor_layout.addWidget(self.build_battery_monitor_group())
        monitor_layout.addWidget(self.build_obstacle_monitor_group())
        monitor_layout.addWidget(self.build_motor_monitor_group())
        monitor_layout.addWidget(self.build_heartbeat_monitor_group())
        monitor_layout.addWidget(self.build_diagnostic_event_group())

        simulator_panel = QWidget()
        simulator_layout = QVBoxLayout(simulator_panel)
        simulator_layout.addWidget(self.build_vehicle_command_group())
        simulator_layout.addWidget(self.build_battery_simulator_group())
        simulator_layout.addWidget(self.build_heartbeat_simulator_group())
        simulator_layout.addStretch()

        root_layout.addWidget(self.build_scroll_area(monitor_panel), 2)
        root_layout.addWidget(self.build_scroll_area(simulator_panel), 1)

        return root

    def build_scroll_area(self, content_widget):
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setFrameShape(QScrollArea.NoFrame)
        scroll_area.setWidget(content_widget)
        return scroll_area

    def build_virtual_render_group(self):
        group = QGroupBox('Virtual Render')
        layout = QVBoxLayout(group)
        layout.addWidget(self.simulation_view)
        return group

    def build_dashboard_group(self):
        group = QGroupBox('Dashboard')
        layout = QVBoxLayout(group)
        layout.addWidget(self.dashboard_widget)
        return group

    def build_node_monitor_group(self):
        group = QGroupBox('Node Monitor')
        layout = QVBoxLayout(group)
        layout.addWidget(self.node_list)
        return group

    def build_vehicle_state_group(self):
        group = QGroupBox('Vehicle State')
        layout = QVBoxLayout(group)
        layout.addWidget(self.state_label)
        return group

    def build_battery_monitor_group(self):
        group = QGroupBox('Battery Status')
        layout = QFormLayout(group)
        layout.addRow('SOC', self.battery_soc_label)
        layout.addRow('Voltage', self.battery_voltage_label)
        layout.addRow('Current', self.battery_current_label)
        return group

    def build_obstacle_monitor_group(self):
        group = QGroupBox('Obstacle Info')
        layout = QFormLayout(group)
        layout.addRow('Detected', self.obstacle_detected_label)
        layout.addRow('Distance', self.obstacle_distance_label)
        layout.addRow('Angle', self.obstacle_angle_label)
        return group

    def build_motor_monitor_group(self):
        group = QGroupBox('Motor Status')
        layout = QFormLayout(group)
        layout.addRow('Enabled', self.motor_enabled_label)
        layout.addRow('Target Linear', self.motor_target_linear_label)
        layout.addRow('Current Linear', self.motor_current_linear_label)
        layout.addRow('Target Angular', self.motor_target_angular_label)
        layout.addRow('Current Angular', self.motor_current_angular_label)
        return group

    def build_vehicle_command_group(self):
        group = QGroupBox('Vehicle Commands')
        layout = QVBoxLayout(group)
        layout.addWidget(self.start_mission_button)
        layout.addWidget(self.service_result_label)
        return group

    def build_heartbeat_monitor_group(self):
        group = QGroupBox('Heartbeat Monitor')
        layout = QVBoxLayout(group)
        layout.addWidget(self.heartbeat_table)
        return group

    def build_diagnostic_event_group(self):
        group = QGroupBox('Diagnostic Events')
        layout = QVBoxLayout(group)
        layout.addWidget(self.diagnostic_event_table)
        return group

    def build_battery_simulator_group(self):
        group = QGroupBox('Battery Simulator')
        layout = QFormLayout(group)

        soc_row = QWidget()
        soc_layout = QHBoxLayout(soc_row)
        soc_layout.setContentsMargins(0, 0, 0, 0)
        soc_layout.addWidget(self.soc_slider)
        soc_layout.addWidget(self.soc_value_label)

        layout.addRow('SOC', soc_row)
        layout.addRow('Voltage', self.voltage_spin)
        layout.addRow('Current', self.current_spin)
        layout.addRow(self.publish_battery_button)

        return group

    def build_heartbeat_simulator_group(self):
        group = QGroupBox('Heartbeat Simulator')
        layout = QVBoxLayout(group)
        for checkbox in self.heartbeat_checks.values():
            layout.addWidget(checkbox)
        return group

    def update_node_list(self, node_names):
        self.node_list.clear()
        self.node_list.addItems(node_names)

    def update_vehicle_state(self, state):
        state_name = VEHICLE_STATE_NAMES.get(state, f'UNKNOWN({state})')
        self.state_label.setText(state_name)
        self.simulation_view.set_vehicle_state(state)
        self.dashboard_widget.set_vehicle_state(state)

    def update_battery_status(self, soc, voltage, current):
        self.battery_soc_label.setText(f'{soc:.1f} %')
        self.battery_voltage_label.setText(f'{voltage:.1f} V')
        self.battery_current_label.setText(f'{current:.1f} A')
        self.dashboard_widget.set_battery_status(soc, voltage, current)

    def update_obstacle_info(self, detected, distance, angle):
        self.obstacle_detected_label.setText('YES' if detected else 'NO')
        self.obstacle_distance_label.setText(f'{distance:.2f} m')
        self.obstacle_angle_label.setText(f'{angle:.1f} deg')
        self.simulation_view.set_obstacle(detected, distance, angle)
        self.dashboard_widget.set_obstacle(detected, distance)

    def update_motor_status(
        self,
        target_linear,
        current_linear,
        target_angular,
        current_angular,
        enabled
    ):
        self.motor_enabled_label.setText('YES' if enabled else 'NO')
        self.motor_target_linear_label.setText(f'{target_linear:.2f} m/s')
        self.motor_current_linear_label.setText(f'{current_linear:.2f} m/s')
        self.motor_target_angular_label.setText(f'{target_angular:.2f} rad/s')
        self.motor_current_angular_label.setText(f'{current_angular:.2f} rad/s')
        self.simulation_view.set_motor_status(
            target_linear,
            current_linear,
            target_angular,
            current_angular,
            enabled
        )
        self.dashboard_widget.set_motor_status(
            current_linear,
            current_angular,
            enabled
        )

    def update_heartbeat_status(self, ecu_name, timestamp_text, received_monotonic):
        if ecu_name not in self.heartbeat_rows:
            row = self.heartbeat_table.rowCount()
            self.heartbeat_table.insertRow(row)
            self.heartbeat_table.setItem(row, 0, QTableWidgetItem(ecu_name))
            self.heartbeat_table.setItem(row, 1, QTableWidgetItem('never'))
            self.heartbeat_table.setItem(row, 2, QTableWidgetItem('-'))
            self.heartbeat_rows[ecu_name] = row

        self.last_heartbeat_times[ecu_name] = received_monotonic

        row = self.heartbeat_rows[ecu_name]
        self.heartbeat_table.item(row, 1).setText('0.0 s ago')
        self.heartbeat_table.item(row, 2).setText(timestamp_text)

    def refresh_heartbeat_age(self):
        now = time.monotonic()

        for ecu_name, row in self.heartbeat_rows.items():
            last_seen = self.last_heartbeat_times.get(ecu_name)
            age_item = self.heartbeat_table.item(row, 1)

            if last_seen is None:
                age_item.setText('never')
                age_item.setBackground(Qt.lightGray)
                continue

            age_sec = now - last_seen
            age_item.setText(f'{age_sec:.1f} s ago')

            if age_sec > 3.0:
                age_item.setBackground(Qt.red)
            else:
                age_item.setBackground(Qt.green)

    def add_diagnostic_event(
        self,
        ecu_name,
        severity,
        description,
        received_time_text
    ):
        row = 0
        self.diagnostic_event_table.insertRow(row)

        severity_name = DIAGNOSTIC_SEVERITY_NAMES.get(
            severity,
            f'UNKNOWN({severity})'
        )

        self.diagnostic_event_table.setItem(
            row,
            0,
            QTableWidgetItem(received_time_text)
        )
        self.diagnostic_event_table.setItem(row, 1, QTableWidgetItem(ecu_name))
        self.diagnostic_event_table.setItem(row, 2, QTableWidgetItem(severity_name))
        self.diagnostic_event_table.setItem(row, 3, QTableWidgetItem(description))

        while self.diagnostic_event_table.rowCount() > 20:
            self.diagnostic_event_table.removeRow(
                self.diagnostic_event_table.rowCount() - 1
            )

    def update_soc_label(self, value):
        self.soc_value_label.setText(f'{value} %')

    def publish_battery(self):
        self.ros_node.publish_battery(
            self.soc_slider.value(),
            self.voltage_spin.value(),
            self.current_spin.value()
        )

    def request_start_mission(self):
        self.service_result_label.setText('Start Mission: requesting...')
        self.ros_node.request_start_mission()

    def update_service_result(self, command_name, success, message):
        status = 'OK' if success else 'FAIL'
        self.service_result_label.setText(
            f'{command_name}: {status} - {message}'
        )

    def publish_enabled_heartbeats(self):
        for ecu_name, checkbox in self.heartbeat_checks.items():
            if checkbox.isChecked():
                self.ros_node.publish_heartbeat(ecu_name)


def spin_ros_node(node):
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass


def format_timestamp_ns(timestamp_ns):
    timestamp_ns = int(timestamp_ns) & 0xFFFFFFFFFFFFFFFF

    if timestamp_ns == 0:
        return '-'

    timestamp_sec = timestamp_ns / 1_000_000_000.0

    try:
        return datetime.fromtimestamp(timestamp_sec).strftime(
            '%H:%M:%S.%f'
        )[:-3]
    except (OverflowError, OSError, ValueError):
        return f'{timestamp_sec:.3f} s'


def format_wall_time():
    return datetime.now().strftime('%H:%M:%S.%f')[:-3]


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def distance_2d(x1, y1, x2, y2):
    return math.hypot(x2 - x1, y2 - y1)


def main(args=None):
    rclpy.init(args=args)

    app = QApplication(sys.argv[:1])
    app.setFont(QFont(GUI_FONT_FAMILY, 10))

    gui_signals = GuiSignals()
    ros_node = SdvTestGuiRosNode(gui_signals)
    window = MainWindow(ros_node, gui_signals)
    window.show()

    ros_thread = threading.Thread(
        target=spin_ros_node,
        args=(ros_node,),
        daemon=True
    )
    ros_thread.start()

    exit_code = app.exec_()

    ros_node.destroy_node()
    rclpy.shutdown()
    ros_thread.join(timeout=1.0)

    sys.exit(exit_code)


if __name__ == '__main__':
    main()
