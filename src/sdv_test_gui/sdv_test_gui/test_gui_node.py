import sys
import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sdv_interfaces.msg import BatteryStatus
from sdv_interfaces.msg import Heartbeat
from sdv_interfaces.msg import VehicleState

from PyQt5.QtCore import QObject, QTimer, pyqtSignal
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QSlider,
    QVBoxLayout,
    QWidget,
)
from PyQt5.QtCore import Qt


VEHICLE_STATE_NAMES = {
    VehicleState.INIT: 'INIT',
    VehicleState.READY: 'READY',
    VehicleState.MISSION: 'MISSION',
    VehicleState.LOW_BATTERY: 'LOW_BATTERY',
    VehicleState.FAULT: 'FAULT',
    VehicleState.EMERGENCY: 'EMERGENCY',
}


class SdvTestGuiRosNode(Node):

    def __init__(self, state_signal):
        super().__init__('sdv_test_gui')

        self.state_signal = state_signal

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

    def vehicle_state_callback(self, msg):
        self.state_signal.emit(int(msg.state))

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


class GuiSignals(QObject):
    vehicle_state_received = pyqtSignal(int)


class MainWindow(QMainWindow):

    def __init__(self, ros_node, gui_signals):
        super().__init__()

        self.ros_node = ros_node
        self.gui_signals = gui_signals

        self.setWindowTitle('SDV Test GUI')
        self.resize(420, 360)

        self.state_label = QLabel('INIT')
        self.state_label.setStyleSheet('font-size: 24px; font-weight: 600;')

        self.soc_slider = QSlider(Qt.Horizontal)
        self.soc_slider.setRange(0, 100)
        self.soc_slider.setValue(80)
        self.soc_value_label = QLabel('80 %')
        self.soc_slider.valueChanged.connect(self.update_soc_label)

        self.voltage_spin = QDoubleSpinBox()
        self.voltage_spin.setRange(0.0, 1000.0)
        self.voltage_spin.setValue(48.0)
        self.voltage_spin.setSuffix(' V')

        self.current_spin = QDoubleSpinBox()
        self.current_spin.setRange(-1000.0, 1000.0)
        self.current_spin.setValue(0.0)
        self.current_spin.setSuffix(' A')

        self.publish_battery_button = QPushButton('Publish Battery')
        self.publish_battery_button.clicked.connect(self.publish_battery)

        self.heartbeat_checks = {}
        for ecu_name in (
            'battery_ecu',
            'sensor_ecu',
            'motor_ecu',
            'security_ecu',
        ):
            checkbox = QCheckBox(ecu_name)
            checkbox.setChecked(ecu_name != 'security_ecu')
            self.heartbeat_checks[ecu_name] = checkbox

        self.heartbeat_timer = QTimer(self)
        self.heartbeat_timer.setInterval(1000)
        self.heartbeat_timer.timeout.connect(self.publish_enabled_heartbeats)
        self.heartbeat_timer.start()

        self.gui_signals.vehicle_state_received.connect(self.update_vehicle_state)

        self.setCentralWidget(self.build_layout())

    def build_layout(self):
        root = QWidget()
        root_layout = QVBoxLayout(root)

        state_group = QGroupBox('Vehicle State')
        state_layout = QVBoxLayout(state_group)
        state_layout.addWidget(self.state_label)

        battery_group = QGroupBox('Battery Simulator')
        battery_layout = QFormLayout(battery_group)

        soc_row = QWidget()
        soc_layout = QHBoxLayout(soc_row)
        soc_layout.setContentsMargins(0, 0, 0, 0)
        soc_layout.addWidget(self.soc_slider)
        soc_layout.addWidget(self.soc_value_label)

        battery_layout.addRow('SOC', soc_row)
        battery_layout.addRow('Voltage', self.voltage_spin)
        battery_layout.addRow('Current', self.current_spin)
        battery_layout.addRow(self.publish_battery_button)

        heartbeat_group = QGroupBox('Heartbeat Simulator')
        heartbeat_layout = QVBoxLayout(heartbeat_group)
        for checkbox in self.heartbeat_checks.values():
            heartbeat_layout.addWidget(checkbox)

        root_layout.addWidget(state_group)
        root_layout.addWidget(battery_group)
        root_layout.addWidget(heartbeat_group)
        root_layout.addStretch()

        return root

    def update_soc_label(self, value):
        self.soc_value_label.setText(f'{value} %')

    def update_vehicle_state(self, state):
        state_name = VEHICLE_STATE_NAMES.get(state, f'UNKNOWN({state})')
        self.state_label.setText(state_name)

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


def main(args=None):
    rclpy.init(args=args)

    app = QApplication(sys.argv[:1])

    gui_signals = GuiSignals()
    ros_node = SdvTestGuiRosNode(gui_signals.vehicle_state_received)
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
