import sys
import threading
import time
from datetime import datetime

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sdv_interfaces.msg import BatteryStatus
from sdv_interfaces.msg import DiagnosticEvent
from sdv_interfaces.msg import Heartbeat
from sdv_interfaces.msg import VehicleState

from PyQt5.QtCore import QObject, Qt, QTimer, pyqtSignal
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
    QSlider,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

MAIN_WINDOW_WIDTH = 1100
MAIN_WINDOW_HEIGHT = 760

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


class GuiSignals(QObject):
    node_list_received = pyqtSignal(object)
    vehicle_state_received = pyqtSignal(int)
    battery_status_received = pyqtSignal(float, float, float)
    heartbeat_received = pyqtSignal(str, str, float)
    diagnostic_event_received = pyqtSignal(str, int, str, str)


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
        self.node_list = QListWidget()

        self.state_label = QLabel('INIT')
        self.state_label.setStyleSheet('font-size: 24px; font-weight: 600;')

        self.battery_soc_label = QLabel('-')
        self.battery_voltage_label = QLabel('-')
        self.battery_current_label = QLabel('-')

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

        self.heartbeat_checks = {}
        for ecu_name in ECU_NAMES:
            checkbox = QCheckBox(ecu_name)
            checkbox.setChecked(False)
            self.heartbeat_checks[ecu_name] = checkbox

    def connect_signals(self):
        self.gui_signals.node_list_received.connect(self.update_node_list)
        self.gui_signals.vehicle_state_received.connect(self.update_vehicle_state)
        self.gui_signals.battery_status_received.connect(self.update_battery_status)
        self.gui_signals.heartbeat_received.connect(self.update_heartbeat_status)
        self.gui_signals.diagnostic_event_received.connect(
            self.add_diagnostic_event
        )

        self.soc_slider.valueChanged.connect(self.update_soc_label)
        self.publish_battery_button.clicked.connect(self.publish_battery)

    def build_layout(self):
        root = QWidget()
        root_layout = QHBoxLayout(root)

        monitor_panel = QWidget()
        monitor_layout = QVBoxLayout(monitor_panel)
        monitor_layout.addWidget(self.build_node_monitor_group())
        monitor_layout.addWidget(self.build_vehicle_state_group())
        monitor_layout.addWidget(self.build_battery_monitor_group())
        monitor_layout.addWidget(self.build_heartbeat_monitor_group())
        monitor_layout.addWidget(self.build_diagnostic_event_group())

        simulator_panel = QWidget()
        simulator_layout = QVBoxLayout(simulator_panel)
        simulator_layout.addWidget(self.build_battery_simulator_group())
        simulator_layout.addWidget(self.build_heartbeat_simulator_group())
        simulator_layout.addStretch()

        root_layout.addWidget(monitor_panel, 2)
        root_layout.addWidget(simulator_panel, 1)

        return root

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

    def update_battery_status(self, soc, voltage, current):
        self.battery_soc_label.setText(f'{soc:.1f} %')
        self.battery_voltage_label.setText(f'{voltage:.1f} V')
        self.battery_current_label.setText(f'{current:.1f} A')

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


def main(args=None):
    rclpy.init(args=args)

    app = QApplication(sys.argv[:1])

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
