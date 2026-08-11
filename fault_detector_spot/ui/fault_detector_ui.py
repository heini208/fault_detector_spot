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
    ApplicationCommandState,
    OperationalIntent,
    StringArray,
    TagElementArray,
)
from fault_detector_spot.shared.ros.qos_profiles import (
    LATCHED_QOS,
)
from rclpy.node import Node

from .inspection.controls import InspectionControls
from .manipulation.controls import ManipulationControls
from .navigation.base_movement_controls import BaseMovementControls
from .navigation.controls import NavigationControls
from .recording.controls import RecordingControls
from .ros.application_client import ApplicationClient
from .ros.navigation_setup_client import NavigationSetupClient
from .shared.status_overview_panel import StatusOverviewPanel


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
        self.application_client = None
        self.navigation_setup_client = None
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
        self.navigation_setup_timer = QTimer(self)
        self.navigation_setup_timer.timeout.connect(
            self._open_navigation_setup
        )
        self.navigation_setup_timer.start(500)

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
            self.handle_emergency_stop
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
        self.application_client = ApplicationClient(self.node)
        self.application_client.state_changed.connect(
            self._process_application_state
        )
        self.application_client.request_rejected.connect(
            self._process_application_error
        )
        self.application_client.emergency_stop_finished.connect(
            self._process_emergency_stop_result
        )
        self.navigation_setup_client = NavigationSetupClient(
            self.node,
            self.application_client.client_id,
        )
        self.navigation_setup_client.state_changed.connect(
            self._process_navigation_setup_state
        )
        self.navigation_setup_client.request_rejected.connect(
            self._process_application_error
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

    def _process_application_state(self, state):
        state_names = {
            ApplicationCommandState.STATE_QUEUED: "QUEUED",
            ApplicationCommandState.STATE_DISPATCHED: "DISPATCHED",
            ApplicationCommandState.STATE_RUNNING: "RUNNING",
            ApplicationCommandState.STATE_SUCCEEDED: "SUCCEEDED",
            ApplicationCommandState.STATE_FAILED: "FAILED",
            ApplicationCommandState.STATE_CANCELLED: "CANCELLED",
        }
        intent_names = {
            value: name.removeprefix("INTENT_")
            for name, value in vars(OperationalIntent).items()
            if name.startswith("INTENT_")
        }
        intent_names[OperationalIntent.INTENT_UNSPECIFIED] = "INTERNAL"
        intent_name = intent_names.get(state.intent, "INTERNAL")
        state_name = state_names.get(state.state, "UNKNOWN")
        self.command_status_label.setText(
            f"Command: {intent_name} {state_name}"
        )
        self.command_status_label.setToolTip(state.detail)
        self._buffer_text = (
            f"Buffer: {state.buffered_command_count} pending"
        )
        self.buffer_label.setToolTip(self._buffer_text)
        self._refresh_buffer_label()
        if hasattr(self, "inspection_controls"):
            self.inspection_controls.handle_application_state(state)

    def _process_application_error(self, detail):
        self.status_label.setText(f"Operation rejected: {detail}")

    def _open_navigation_setup(self):
        if self.navigation_setup_client is None:
            return
        if self.navigation_setup_client.context_id:
            self.navigation_setup_timer.stop()
            return
        if self.navigation_setup_client.open() is not None:
            self.navigation_setup_timer.stop()

    def _process_navigation_setup_state(self, state):
        if hasattr(self, "navigation_controls"):
            self.navigation_controls.apply_setup_state(state)
        self.status_label.setText(state.detail)

    def execute_navigation_setup(self, intent):
        if self.navigation_setup_client is None:
            self._process_application_error("ROS is unavailable")
            return None
        request_id = self.navigation_setup_client.execute(intent)
        if request_id is not None:
            self.status_label.setText("Navigation setup request submitted")
        return request_id

    def _process_emergency_stop_result(self, accepted, detail):
        prefix = "Emergency stop accepted" if accepted else "Emergency stop failed"
        self.status_label.setText(f"{prefix}: {detail}")

    def _process_available_frames(self, msg: StringArray):
        self.available_frames = list(msg.names)
        self.manipulation_controls.update_frames_dropdown()
        self.base_movement_controls.update_frames_dropdown()

    def execute_operation(self, intent, context_id=""):
        if self.application_client is None:
            self._process_application_error("ROS is unavailable")
            return None
        local_id = self.application_client.execute(intent, context_id)
        if local_id is not None:
            self.status_label.setText("Operational request submitted")
        return local_id

    def handle_simple_operation(self, intent_id):
        intent = OperationalIntent()
        intent.intent = intent_id
        return self.execute_operation(intent)

    def handle_emergency_stop(self):
        if self.application_client is None:
            self._process_application_error("ROS is unavailable")
            return None
        self.status_label.setText("Emergency stop requested")
        return self.application_client.emergency_stop()

    def show_setup_unavailable(self, workflow):
        detail = f"{workflow} is pending its coordinator API"
        self.status_label.setText(detail)
        return False

    def closeEvent(self, event):
        self.timer.stop()
        self.navigation_setup_timer.stop()
        if self.navigation_setup_client is not None:
            self.navigation_setup_client.close()
            self.navigation_setup_client.destroy()
        if self.application_client is not None:
            self.application_client.destroy()
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
