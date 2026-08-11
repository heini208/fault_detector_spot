#!/usr/bin/env python3
import signal
import sys

from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QColor, QFont, QFontMetrics
from PyQt5.QtWidgets import (
    QApplication,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

import rclpy
from fault_detector_msgs.msg import (
    BasicCommand,
    CommandStatus,
    ComplexCommand,
    StringArray,
    TagElementArray,
)
from fault_detector_spot.common.ros.qos_profiles import (
    COMMAND_QOS,
    LATCHED_QOS,
)
from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.data.request_identity import new_request_id
from rclpy.node import Node
from std_msgs.msg import Header, String

from .controls.base_movement_controls import BaseMovementControls
from .controls.inspection_controls import InspectionControls
from .controls.manipulation_controls import ManipulationControls
from .controls.navigation_controls import NavigationControls
from .controls.recording_controls import RecordingControls
from .widgets.status_overview_panel import StatusOverviewPanel


class Fault_Detector_UI(QWidget):
    def __init__(self, node: Node = None):
        super().__init__()
        self.node = node
        self.setWindowTitle("Fault Detector Spot")
        self.resize(700, 600)

        self.status_label = QLabel("Status: Waiting for connection")
        self.buffer_label = QLabel("Buffer: []")
        self.command_status_label = QLabel("Command: IDLE")
        self.visible_label = QLabel("Visible tags: []")
        self.visible_label.setTextFormat(Qt.RichText)
        self.navigation_mode_label = QLabel("Navigation: OFF")
        self._buffer_text = "Buffer: []"

        self.visible_tags = {}
        self.base_tags = {}
        self.reachable_tags = {}
        self.available_frames = []
        self.inspection_object_root = self._inspection_root_parameter()

        if self.node:
            self.init_ros_communication()

        self.manipulation_controls = ManipulationControls(self)
        self.recording_controls = RecordingControls(self)
        self.navigation_controls = NavigationControls(self)
        self.base_movement_controls = BaseMovementControls(self)
        self.inspection_controls = InspectionControls(self)

        self.create_user_interface()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._spin_and_refresh)
        self.timer.start(10)

    def create_user_interface(self):
        main_layout = QVBoxLayout(self)
        main_layout.setSpacing(6)

        self.status_overview_panel = StatusOverviewPanel(
            self.status_label,
            self.command_status_label,
            self.navigation_mode_label,
            self.visible_label,
            self.buffer_label,
            self._make_estop_button(),
            self,
        )
        main_layout.addWidget(self.status_overview_panel)

        self.tabs = QTabWidget()
        self.tabs.currentChanged.connect(self._on_tab_changed)
        main_layout.addWidget(self.tabs)
        self.add_manipulator_control_tab()
        self.add_base_movement_control_tab()
        self.add_navigation_control_tab()
        self.add_inspection_control_tab()

        self.recording_controls.add_rows(main_layout)
        QTimer.singleShot(0, self._refresh_buffer_label)

    def set_navigation_mode(self, active: bool):
        text = "ON" if active else "OFF"
        self.navigation_mode_label.setText(f"Navigation: {text}")

    def _on_tab_changed(self, index):
        if self.tabs.tabText(index) == "Navigation Control":
            self.navigation_controls._apply_map_list()
            self.navigation_controls._apply_waypoint_list()
        if self.tabs.tabText(index) == "Inspection Control":
            self.inspection_controls.refresh_saved_definitions()

    def _inspection_root_parameter(self):
        if self.node is None:
            return None
        parameter_name = "inspection.object_root"
        if not self.node.has_parameter(parameter_name):
            self.node.declare_parameter(parameter_name, "")
        value = self.node.get_parameter(parameter_name).value
        return value.strip() or None

    def _make_estop_button(self) -> QPushButton:
        if hasattr(self, "estop_button"):
            return self.estop_button

        self.estop_button = QPushButton("EMERGENCY STOP")
        self.estop_button.setStyleSheet(
            """
            QPushButton {
                background-color: #C62828;
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
            """
        )

        font = QFont()
        font.setPointSize(14)
        font.setBold(True)
        self.estop_button.setFont(font)
        self.estop_button.clicked.connect(
            lambda _, cid=CommandID.EMERGENCY_CANCEL:
            self.handle_simple_command(cid)
        )
        return self.estop_button

    def _make_estop_row(self) -> QHBoxLayout:
        row = QHBoxLayout()
        row.addWidget(
            self._make_estop_button(),
            alignment=Qt.AlignRight,
        )
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

    def add_inspection_control_tab(self):
        inspection_tab = QWidget()
        inspection_layout = QVBoxLayout(inspection_tab)
        self.inspection_controls.add_rows(inspection_layout)
        inspection_layout.addStretch()
        self.tabs.addTab(inspection_tab, "Inspection Control")

    def init_ros_communication(self):
        self.complex_command_publisher = self.node.create_publisher(
            ComplexCommand,
            "fault_detector/commands/complex_command",
            COMMAND_QOS,
        )

        self.visible_tags_sub = self.node.create_subscription(
            TagElementArray,
            "fault_detector/state/visible_tags",
            self._process_visible_tags,
            10,
        )

        self.base_tags_sub = self.node.create_subscription(
            TagElementArray,
            "fault_detector/state/base_tags",
            self._process_base_tags,
            10,
        )

        self.reachable_tags_sub = self.node.create_subscription(
            TagElementArray,
            "fault_detector/state/reachable_tags",
            self._process_reachable_tags,
            10,
        )

        self.buffer_sub = self.node.create_subscription(
            String,
            "fault_detector/command_buffer",
            self._process_buffer,
            10,
        )
        self.cmd_status_sub = self.node.create_subscription(
            String,
            "fault_detector/command_tree_status",
            self._process_command_status,
            10,
        )
        self.structured_cmd_status_sub = self.node.create_subscription(
            CommandStatus,
            "fault_detector/command_status",
            self._process_structured_command_status,
            10,
        )
        self.available_frames_sub = self.node.create_subscription(
            StringArray,
            "fault_detector/state/available_frames",
            self._process_available_frames,
            LATCHED_QOS,
        )

        self.status_label.setText("Status: Connected to ROS2")

    def _spin_and_refresh(self):
        if self.node:
            rclpy.spin_once(self.node, timeout_sec=0.001)
        parts = []
        for tag_id in sorted(self.visible_tags.keys()):
            color = "green" if tag_id in self.reachable_tags else "red"
            parts.append(
                f'<span style="color:{color}">{tag_id}</span>'
            )

        html = "Visible tags: [" + ", ".join(parts) + "]"
        self.visible_label.setText(html)

    def _process_visible_tags(self, msg: TagElementArray):
        self.visible_tags = {tag.id: tag for tag in msg.elements}

    def _process_base_tags(self, msg: TagElementArray):
        self.base_tags = {tag.id: tag for tag in msg.elements}
        if hasattr(self, "inspection_controls"):
            self.inspection_controls.handle_base_tags(msg.elements)

    def _process_reachable_tags(self, msg: TagElementArray):
        self.reachable_tags = {tag.id: tag for tag in msg.elements}

    def _process_buffer(self, msg: String):
        self._buffer_text = f"Buffer: {msg.data}"
        self.buffer_label.setToolTip(self._buffer_text)
        self._refresh_buffer_label()

    def _refresh_buffer_label(self):
        if not hasattr(self, "buffer_label"):
            return
        available_width = max(220, self.buffer_label.width() - 8)
        metrics = QFontMetrics(self.buffer_label.font())
        self.buffer_label.setText(
            metrics.elidedText(
                self._buffer_text,
                Qt.ElideRight,
                available_width,
            )
        )

    def _process_command_status(self, msg: String):
        self.command_status_label.setText(f"Command: {msg.data}")

    def _process_structured_command_status(
        self,
        msg: CommandStatus,
    ):
        self.inspection_controls.handle_command_status(msg)

    def _process_available_frames(self, msg: StringArray):
        self.available_frames = list(msg.names)
        self.manipulation_controls.update_frames_dropdown()
        self.base_movement_controls.update_frames_dropdown()

    def build_basic_command(self, command_id: str) -> BasicCommand:
        cmd = BasicCommand()
        cmd.header = Header()
        cmd.header.stamp = self.node.get_clock().now().to_msg()
        cmd.command_id = command_id
        cmd.request_id = new_request_id()
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

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if hasattr(self, "buffer_label"):
            self._refresh_buffer_label()

    def update_frames_dropdown(self, frames_dropdown):
        previous_selection = frames_dropdown.currentText()

        frames_dropdown.blockSignals(True)
        frames_dropdown.clear()

        available_frames = list(self.available_frames)
        frames = list(available_frames)

        if (
            previous_selection
            and previous_selection not in frames
            and previous_selection != "no frames available"
        ):
            frames.append(previous_selection)

        if not frames:
            frames = ["no frames available"]

        missing_indexes = set()

        for frame in frames:
            frames_dropdown.addItem(frame)
            index = frames_dropdown.count() - 1
            if (
                frame not in available_frames
                and frame != "no frames available"
            ):
                frames_dropdown.setItemData(
                    index,
                    QColor("red"),
                    Qt.ForegroundRole,
                )
                missing_indexes.add(index)

        target_index = frames_dropdown.findText(previous_selection)
        if target_index < 0:
            target_index = 0
        frames_dropdown.setCurrentIndex(target_index)

        if target_index in missing_indexes:
            frames_dropdown.setItemData(
                target_index,
                QColor("red"),
                Qt.ForegroundRole,
            )

        frames_dropdown.blockSignals(False)


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
    """Raised when the requested tag ID is not visible."""

    pass
