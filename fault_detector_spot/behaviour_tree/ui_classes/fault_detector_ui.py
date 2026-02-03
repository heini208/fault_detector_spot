#!/usr/bin/env python3
import signal
import sys

from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont, QFontMetrics
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QPushButton, QApplication, QTabWidget
)

import rclpy
from fault_detector_msgs.msg import ComplexCommand, BasicCommand, TagElementArray, StringArray
from fault_detector_spot.behaviour_tree.QOS_PROFILES import COMMAND_QOS, LATCHED_QOS
from fault_detector_spot.behaviour_tree.commands.command_ids import CommandID
from rclpy.node import Node
from std_msgs.msg import Header, String
from .base_movement_controls import BaseMovementControls
from .manipulation_controls import ManipulationControls
from .navigation_controls import NavigationControls
from .recording_controls import RecordingControls


class Fault_Detector_UI(QWidget):
    def __init__(self, node: Node = None):
        super().__init__()
        self.node = node
        self.setWindowTitle("Fault Detector Spot")
        self.resize(700, 600)

        self.status_label = QLabel("Status: Waiting for connection")
        self.buffer_label = QLabel("Buffer: []")
        self.command_status_label = QLabel("Command Status: IDLE")
        self.visible_label = QLabel("Visible tags: []")
        self.visible_label.setTextFormat(Qt.RichText)
        self.navigation_mode_label = QLabel("Navigation: OFF")

        self.visible_tags = {}
        self.reachable_tags = {}
        self.available_frames = []

        if self.node:
            self.init_ros_communication()

        # Initialize control objects
        self.manipulation_controls = ManipulationControls(self)
        self.recording_controls = RecordingControls(self)
        self.navigation_controls = NavigationControls(self)
        self.base_movement_controls = BaseMovementControls(self)

        self.create_user_interface()

        # periodically spin ROS and refresh UI
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._spin_and_refresh)
        self.timer.start(10)



    # ------ build UI
    def create_user_interface(self):
        main_layout = QVBoxLayout(self)
        top_row = QHBoxLayout()
        status_col = QVBoxLayout()
        status_col.addWidget(self.status_label)
        status_col.addWidget(self.buffer_label)
        status_col.addWidget(self.command_status_label)
        status_col.addWidget(self.navigation_mode_label)

        self.visible_label = QLabel()
        self.visible_label.setTextFormat(Qt.RichText)
        self.visible_label.setText("Visible tags: []")
        status_col.addWidget(self.visible_label)

        top_row.addLayout(status_col)
        top_row.addStretch()
        top_row.addLayout(self._make_estop_row())

        main_layout.addLayout(top_row)

        # Tabs
        self.tabs = QTabWidget()
        self.tabs.currentChanged.connect(self._on_tab_changed)
        main_layout.addWidget(self.tabs)
        self.add_manipulator_control_tab()
        self.add_base_movement_control_tab()
        self.add_navigation_control_tab()

        self.recording_controls.add_rows(main_layout)

    def set_navigation_mode(self, active: bool):
        text = "ON" if active else "OFF"
        self.navigation_mode_label.setText(f"Navigation: {text}")

    def _on_tab_changed(self, index):
        if self.tabs.tabText(index) == "Navigation Control":
            self.navigation_controls._apply_map_list()
            self.navigation_controls._apply_waypoint_list()

    def _make_estop_row(self) -> QHBoxLayout:
        row = QHBoxLayout()
        self.estop_button = QPushButton("EMERGENCY STOP")

        self.estop_button.setStyleSheet("""
            QPushButton {
                background-color: #C62828;   /* deep red */
                color: white;
                font-weight: bold;
                border: none;
                border-radius: 8px;
                padding: 10px 20px;
            }
            QPushButton:hover {
                background-color: #B71C1C;
            }
            QPushButton:pressed {
                background-color: #8E0000;
            }
        """)

        font = QFont()
        font.setPointSize(14)
        font.setBold(True)
        self.estop_button.setFont(font)

        self.estop_button.clicked.connect(
            lambda _, cid=CommandID.EMERGENCY_CANCEL: self.handle_simple_command(cid)
        )

        row.addWidget(self.estop_button, alignment=Qt.AlignRight)
        return row

    def add_manipulator_control_tab(self):
        manip_tab = QWidget()
        manip_layout = QVBoxLayout(manip_tab)
        self.manipulation_controls.add_rows(manip_layout)
        self.tabs.addTab(manip_tab, "Manipulation Control")

    def add_navigation_control_tab(self):
        nav_tab = QWidget()
        nav_layout = QVBoxLayout(nav_tab)
        self.navigation_controls.add_rows(nav_layout)
        self.tabs.addTab(nav_tab, "Navigation Control")

    def add_base_movement_control_tab(self):
        base_tab = QWidget()
        base_layout = QVBoxLayout(base_tab)
        self.base_movement_controls.add_rows(base_layout)
        self.tabs.addTab(base_tab, "Base Movement Control")

    # ---- feedback information

    def init_ros_communication(self):
        self.complex_command_publisher = self.node.create_publisher(
            ComplexCommand, "fault_detector/commands/complex_command", COMMAND_QOS
        )

        self.visible_tags_sub = self.node.create_subscription(
            TagElementArray,
            "fault_detector/state/visible_tags",
            self._process_visible_tags,
            10
        )

        self.reachable_tags_sub = self.node.create_subscription(
            TagElementArray,
            "fault_detector/state/reachable_tags",
            self._process_reachable_tags,
            10
        )

        self.buffer_sub = self.node.create_subscription(
            String,
            "fault_detector/command_buffer",
            self._process_buffer,
            10
        )
        self.cmd_status_sub = self.node.create_subscription(
            String,
            "fault_detector/command_tree_status",
            self._process_command_status,
            10
        )
        self.available_frames_sub = self.node.create_subscription(
            StringArray,
            "fault_detector/state/available_frames",
            self._process_available_frames,
            LATCHED_QOS
        )

        self.status_label.setText("Status: Connected to ROS2")

    def _spin_and_refresh(self):
        if self.node:
            rclpy.spin_once(self.node, timeout_sec=0.001)
        parts = []
        for tid in sorted(self.visible_tags.keys()):
            color = "green" if tid in self.reachable_tags else "red"
            parts.append(f'<span style="color:{color}">{tid}</span>')

        html = "Visible tags: [" + ", ".join(parts) + "]"
        self.visible_label.setText(html)

    def _process_visible_tags(self, msg: TagElementArray):
        self.visible_tags = {tag.id: tag for tag in msg.elements}

    def _process_reachable_tags(self, msg: TagElementArray):
        self.reachable_tags = {tag.id: tag for tag in msg.elements}

    def _process_buffer(self, msg: String):
        max_width = 400
        text = f"Buffer: {msg.data}"
        metrics = QFontMetrics(self.buffer_label.font())
        elided = metrics.elidedText(text, Qt.ElideRight, max_width)
        self.buffer_label.setText(elided)
        self.buffer_label.setFixedWidth(max_width)

    def _process_command_status(self, msg: String):
        self.command_status_label.setText(f"Command Status: {msg.data}")

    def _process_available_frames(self, msg: StringArray):
        self.available_frames = list(msg.names)
        self.manipulation_controls.update_frames_dropdown()
        self.base_movement_controls.update_frames_dropdown()

    def build_basic_command(self, command_id: str) -> BasicCommand:
        cmd = BasicCommand()
        cmd.header = Header()
        cmd.header.stamp = self.node.get_clock().now().to_msg()
        cmd.command_id = command_id
        return cmd

    def handle_simple_command(self, command_id: str):
        cmd = self.build_basic_command(command_id)
        as_complex_command = ComplexCommand()
        as_complex_command.command = cmd
        self.complex_command_publisher.publish(as_complex_command)
        self.status_label.setText(f"Command sent: {command_id}")

    def closeEvent(self, event):
        self.timer.stop()
        event.accept()


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("fault_detector_ui_node")
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    app = QApplication(sys.argv)
    ui = Fault_Detector_UI(node)
    ui.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()


class TagNotFound(Exception):
    """Raised when the requested tag ID isn’t visible."""
    pass