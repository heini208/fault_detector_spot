#!/usr/bin/env python3
import signal
import sys
from pathlib import Path

from PyQt5.QtCore import QTimer, Qt, QUrl
from PyQt5.QtGui import QColor, QDesktopServices, QFont, QFontMetrics
from PyQt5.QtWidgets import (
    QApplication,
    QHBoxLayout,
    QLabel,
    QMessageBox,
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
from fault_detector_spot.shared.persistence.runtime_paths import (
    default_measurement_root,
)
from rclpy.node import Node

from .inspection.finalizing_controls import FinalizingInspectionControls
from .manipulation.controls import ManipulationControls
from .navigation.base_movement_controls import BaseMovementControls
from .navigation.controls import NavigationControls
from .recording.controls import RecordingControls
from .ros.application_client import ApplicationClient
from .ros.micro_ros_agent_status_client import MicroRosAgentStatusClient
from .ros.navigation_setup_client import NavigationSetupClient
from .ros.probe_setup_client import ProbeSetupClient
from .ros.sensor_attachment_client import SensorAttachmentClient
from .ros.sensor_acquisition_client import SensorAcquisitionClient
from .ros.sensor_head_connection_client import SensorHeadConnectionClient
from .ros.sensor_registry_client import SensorRegistryClient
from .ros.sensor_topic_suggestion_client import (
    SensorTopicSuggestionClient,
)
from .sensor.controls import SensorControls
from .sensor.models import (
    SensorAcquisitionViewStatus,
    SensorAttachmentViewStatus,
    SensorHeadConnectionViewStatus,
)
from .shared.status_overview_panel import StatusOverviewPanel


_SENSOR_RECORDING_PRESENTATIONS = {
    SensorAcquisitionViewStatus.IDLE: ("#757575", "Idle"),
    SensorAcquisitionViewStatus.STARTING: ("#EF6C00", "Starting"),
    SensorAcquisitionViewStatus.RECORDING: ("#2E7D32", "Recording"),
    SensorAcquisitionViewStatus.STOPPING: ("#EF6C00", "Stopping"),
    SensorAcquisitionViewStatus.FAILED: ("#C62828", "Failed"),
    SensorAcquisitionViewStatus.UNAVAILABLE: ("#757575", "Unavailable"),
}


class Fault_Detector_UI(QWidget):
    def __init__(self, node: Node = None):
        super().__init__()
        self.node = node
        self.setWindowTitle("Fault Detector Spot")
        self.resize(700, 600)
        self.measurement_root = self._measurement_root_parameter()

        self.status_label = QLabel("Status: Waiting for connection")
        self.buffer_label = QLabel("Buffer: []")
        self.command_status_label = QLabel("Command: IDLE")
        self.visible_label = QLabel("Visible tags: []")
        self.visible_label.setTextFormat(Qt.RichText)
        self.navigation_mode_label = QLabel("Navigation: OFF")
        self.sensor_indicator_label = QLabel("●")
        self.sensor_indicator_label.setAlignment(Qt.AlignCenter)
        self.sensor_indicator_label.setFixedWidth(12)
        self.sensor_status_label = QLabel("Unknown")
        self.sensor_recording_indicator_label = QLabel("●")
        self.sensor_recording_indicator_label.setAlignment(Qt.AlignCenter)
        self.sensor_recording_indicator_label.setFixedWidth(12)
        self.sensor_recording_button = QPushButton("Record")
        self.sensor_recording_button.clicked.connect(
            self._toggle_sensor_recording
        )
        self.open_measurements_button = QPushButton("Folder")
        self.open_measurements_button.setToolTip(
            "Open all saved sensor measurements"
        )
        self.open_measurements_button.clicked.connect(
            self.open_measurement_directory
        )
        self.sensor_confirm_button = QPushButton("✓")
        self.sensor_confirm_button.setFixedSize(24, 24)
        self.sensor_confirm_button.setToolTip(
            "Confirm the displayed physical sensor state"
        )
        self.sensor_confirm_button.setEnabled(False)
        self.sensor_confirm_button.clicked.connect(
            self._request_sensor_state_confirmation
        )
        self.sensor_connection_indicator_label = QLabel("●")
        self.sensor_connection_indicator_label.setAlignment(Qt.AlignCenter)
        self.sensor_connection_indicator_label.setFixedWidth(12)
        self.sensor_connection_status_label = QLabel("Unknown")
        self.agent_indicator_label = QLabel("●")
        self.agent_indicator_label.setAlignment(Qt.AlignCenter)
        self.agent_indicator_label.setFixedWidth(12)
        self.agent_endpoint_button = QPushButton("Show IP")
        self.agent_endpoint_button.setCheckable(True)
        self.agent_endpoint_button.setEnabled(False)
        self.agent_endpoint_button.toggled.connect(
            self._set_agent_endpoint_visibility
        )
        self.agent_copy_button = QPushButton("Copy")
        self.agent_copy_button.setVisible(False)
        self.agent_copy_button.clicked.connect(
            self._copy_agent_configuration
        )
        self._agent_endpoint_text = "IP unavailable"
        self._agent_command = ""
        self._agent_endpoint_visible = False
        self._agent_status_stale = True
        self.set_micro_ros_agent_status(None)
        self.set_sensor_status("unknown")
        self.set_sensor_connection_status("unknown")
        self.set_sensor_acquisition_state(None)
        self._buffer_text = "Buffer: []"

        self.visible_tags = {}
        self.reachable_tags = {}
        self.available_frames = []
        self.application_client = None
        self.navigation_setup_client = None
        self.probe_setup_client = None
        self.sensor_attachment_client = None
        self.sensor_registry_client = None
        self.sensor_topic_suggestion_client = None
        self.sensor_head_connection_client = None
        self.sensor_acquisition_client = None
        self.micro_ros_agent_status_client = None
        self._sensor_definitions = {}
        self._sensor_topic_suggestions = ()
        self._sensor_attachment_state = None
        self._sensor_head_connection_state = None
        self._sensor_connection_stale = True
        self.inspection_object_root = self._inspection_root_parameter()

        if self.node:
            self.init_ros_communication()

        self.manipulation_controls = ManipulationControls(self)
        self.recording_controls = RecordingControls(self)
        self.navigation_controls = NavigationControls(self)
        self.base_movement_controls = BaseMovementControls(self)
        self.inspection_controls = FinalizingInspectionControls(self)
        self.sensor_controls = SensorControls(self)
        self.sensor_controls.select_requested.connect(
            self._select_sensor_attachment
        )
        self.sensor_controls.create_requested.connect(
            self._create_sensor_definition
        )
        self.sensor_controls.update_requested.connect(
            self._update_sensor_definition
        )
        self.sensor_controls.delete_requested.connect(
            self._delete_sensor_definition
        )
        self.sensor_controls.apply_topic_suggestions(
            self._sensor_topic_suggestions
        )
        self.create_user_interface()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._spin_and_refresh)
        self.timer.start(10)
        self.navigation_setup_timer = QTimer(self)
        self.navigation_setup_timer.timeout.connect(
            self._open_navigation_setup
        )
        self.navigation_setup_timer.start(500)
        self.probe_setup_timer = QTimer(self)
        self.probe_setup_timer.timeout.connect(self._open_probe_setup)
        self.probe_setup_timer.start(500)

    def create_user_interface(self):
        main_layout = QVBoxLayout(self)
        main_layout.setSpacing(6)

        self.status_overview_panel = StatusOverviewPanel(
            self.status_label,
            self.command_status_label,
            self.navigation_mode_label,
            self.visible_label,
            self.buffer_label,
            self.sensor_indicator_label,
            self.sensor_status_label,
            self.sensor_confirm_button,
            self.sensor_recording_indicator_label,
            self.sensor_recording_button,
            self.open_measurements_button,
            self.sensor_connection_indicator_label,
            self.sensor_connection_status_label,
            self.agent_indicator_label,
            self.agent_endpoint_button,
            self.agent_copy_button,
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
        self.add_sensor_control_tab()

        self.recording_controls.add_rows(main_layout)
        QTimer.singleShot(0, self._refresh_buffer_label)

    def set_navigation_mode(self, active: bool):
        text = "ON" if active else "OFF"
        self.navigation_mode_label.setText(f"Navigation: {text}")

    def set_sensor_status(
        self,
        status: str,
        sensor_name: str = "",
    ) -> None:
        states = {
            "confirmed": ("#2E7D32", "Confirmed"),
            "pending": ("#EF6C00", "Confirmation pending"),
            "unknown": ("#757575", "Sensor state unavailable"),
        }
        color, tooltip = states.get(status, states["unknown"])
        self.sensor_indicator_label.setStyleSheet(
            f"color: {color}; font-size: 14px;"
        )
        self.sensor_indicator_label.setToolTip(tooltip)
        self.sensor_status_label.setText(sensor_name.strip() or "Unknown")

    def set_micro_ros_agent_status(
        self,
        running,
        address: str = "",
        port: int = 0,
        detail: str = "Agent status unavailable",
    ) -> None:
        if running is True:
            color = "#2E7D32"
            state_text = "Running"
        elif running is False:
            color = "#C62828"
            state_text = "Not running"
        else:
            color = "#757575"
            state_text = "Status unavailable"
        self.agent_indicator_label.setStyleSheet(
            f"color: {color}; font-size: 14px;"
        )
        self.agent_indicator_label.setToolTip(
            f"{state_text}. {detail}".strip()
        )

        normalized_address = address.strip()
        normalized_port = int(port)
        if normalized_address and normalized_port > 0:
            self._agent_endpoint_text = (
                f"{normalized_address}:{normalized_port}"
            )
            self._agent_command = (
                f"set-agent {normalized_address} {normalized_port}"
            )
            self.agent_endpoint_button.setEnabled(True)
        else:
            self._agent_endpoint_text = "IP unavailable"
            self._agent_command = ""
            self._agent_endpoint_visible = False
            self.agent_endpoint_button.setEnabled(False)
            self.agent_endpoint_button.setToolTip(detail)
        self._refresh_agent_endpoint_visibility()

    def set_sensor_connection_status(
        self,
        status: str,
        text: str = "Unknown",
        detail: str = "Sensor-head status unavailable",
    ) -> None:
        colors = {
            "connected": "#2E7D32",
            "unassigned": "#EF6C00",
            "mismatch": "#C62828",
            "offline": "#757575",
            "unknown": "#757575",
        }
        color = colors.get(status, colors["unknown"])
        self.sensor_connection_indicator_label.setStyleSheet(
            f"color: {color}; font-size: 14px;"
        )
        self.sensor_connection_indicator_label.setToolTip(detail)
        self.sensor_connection_status_label.setText(text)
        self.sensor_connection_status_label.setToolTip(detail)

    def set_sensor_acquisition_state(self, state) -> None:
        """Render authoritative sensor recording state in the header."""
        self._sensor_acquisition_state = state
        status = (
            state.status
            if state is not None
            else SensorAcquisitionViewStatus.UNAVAILABLE
        )
        color, label = _SENSOR_RECORDING_PRESENTATIONS[status]
        detail = getattr(state, "detail", "")
        self.sensor_recording_indicator_label.setStyleSheet(
            f"color: {color}; font-size: 14px;"
        )
        self.sensor_recording_indicator_label.setToolTip(
            f"{label}. {detail}".strip()
        )
        self.sensor_recording_button.setToolTip(
            f"{label}. {detail}".strip()
        )
        is_recording = status is SensorAcquisitionViewStatus.RECORDING
        is_busy = status in {
            SensorAcquisitionViewStatus.STARTING,
            SensorAcquisitionViewStatus.STOPPING,
            SensorAcquisitionViewStatus.UNAVAILABLE,
        }
        self.sensor_recording_button.setText(
            "Stop" if is_recording else "Record"
        )
        self.sensor_recording_button.setEnabled(not is_busy)

    def _set_agent_endpoint_visibility(self, visible: bool) -> None:
        self._agent_endpoint_visible = bool(visible and self._agent_command)
        self._refresh_agent_endpoint_visibility()

    def _refresh_agent_endpoint_visibility(self) -> None:
        visible = bool(self._agent_endpoint_visible and self._agent_command)
        self.agent_endpoint_button.blockSignals(True)
        self.agent_endpoint_button.setChecked(visible)
        self.agent_endpoint_button.blockSignals(False)
        self.agent_copy_button.setVisible(visible)
        if not self._agent_command:
            self.agent_endpoint_button.setText("IP unavailable")
            return
        if visible:
            self.agent_endpoint_button.setText(self._agent_endpoint_text)
            self.agent_endpoint_button.setToolTip("Click to hide Agent IP")
            self.agent_copy_button.setToolTip(
                f"Copy: {self._agent_command}"
            )
            return
        self.agent_endpoint_button.setText("Show IP")
        self.agent_endpoint_button.setToolTip("Click to show Agent IP")

    def _process_micro_ros_agent_status(self, state) -> None:
        self._agent_status_stale = False
        self.set_micro_ros_agent_status(
            bool(state.running),
            state.advertised_address,
            int(state.port),
            state.detail,
        )

    def _copy_agent_configuration(self) -> None:
        if not self._agent_command:
            return
        QApplication.clipboard().setText(self._agent_command)
        self.agent_copy_button.setText("Copied")
        QTimer.singleShot(1200, self._restore_agent_copy_button_text)

    def _restore_agent_copy_button_text(self) -> None:
        self.agent_copy_button.setText("Copy")

    def _process_sensor_definitions(self, definitions):
        definitions = tuple(definitions)
        self._sensor_definitions = {
            definition.sensor_id: definition
            for definition in definitions
        }
        if hasattr(self, "sensor_controls"):
            self.sensor_controls.apply_definitions(definitions)
        self._refresh_sensor_status()

    def _process_sensor_topic_suggestions(self, suggestions):
        self._sensor_topic_suggestions = tuple(suggestions)
        if hasattr(self, "sensor_controls"):
            self.sensor_controls.apply_topic_suggestions(suggestions)

    def _process_sensor_attachment_state(self, state):
        self._sensor_attachment_state = state
        if hasattr(self, "sensor_controls"):
            self.sensor_controls.apply_attachment_state(state)
        self._refresh_sensor_status()

    def _process_sensor_head_connection_state(self, state):
        self._sensor_connection_stale = False
        self._sensor_head_connection_state = state
        if hasattr(self, "sensor_controls"):
            self.sensor_controls.apply_sensor_head_connection(state)
        self._refresh_sensor_connection_status()

    def _sensor_display_name(self, sensor_id):
        definition = self._sensor_definitions.get(sensor_id)
        if definition is not None:
            return definition.display_name
        return sensor_id

    def _refresh_sensor_status(self):
        state = self._sensor_attachment_state
        if state is None:
            self.set_sensor_status("unknown")
            self.sensor_confirm_button.setEnabled(False)
            return

        if state.status is SensorAttachmentViewStatus.PENDING:
            sensor_name = self._sensor_display_name(
                state.pending_sensor_id
            )
            self.set_sensor_status("pending", sensor_name)
            self.sensor_confirm_button.setEnabled(True)
            return

        if state.status is SensorAttachmentViewStatus.NONE:
            self.set_sensor_status("pending", "No sensor")
            self.sensor_confirm_button.setEnabled(True)
            return

        sensor_name = (
            self._sensor_display_name(state.active_sensor_id)
            if state.active_sensor_id
            else "No sensor"
        )
        self.set_sensor_status("confirmed", sensor_name)
        self.sensor_confirm_button.setEnabled(False)

    def _refresh_sensor_connection_status(self):
        state = self._sensor_head_connection_state
        if state is None:
            self.set_sensor_connection_status("unknown")
            return
        connected = tuple(state.connected_sensor_ids)
        if state.status is SensorHeadConnectionViewStatus.MATCHED:
            self.set_sensor_connection_status(
                "connected", "Connected", state.detail
            )
            return
        if state.status is SensorHeadConnectionViewStatus.UNASSIGNED:
            text = (
                f"Unassigned: {connected[0]}"
                if len(connected) == 1
                else f"{len(connected)} unassigned"
            )
            self.set_sensor_connection_status(
                "unassigned", text, state.detail
            )
            return
        if state.status is SensorHeadConnectionViewStatus.MISMATCH:
            self.set_sensor_connection_status(
                "mismatch", "ID mismatch", state.detail
            )
            return
        if state.status is SensorHeadConnectionViewStatus.NO_HEADS:
            text = "Offline" if state.expected_sensor_id else "No head"
            self.set_sensor_connection_status(
                "offline", text, state.detail
            )
            return
        if (
            state.status
            is SensorHeadConnectionViewStatus.AGENT_UNAVAILABLE
        ):
            self.set_sensor_connection_status(
                "offline", "Agent unavailable", state.detail
            )
            return
        self.set_sensor_connection_status("unknown", detail=state.detail)

    def _request_sensor_state_confirmation(self):
        state = self._sensor_attachment_state
        if state is None:
            return None
        if state.status is SensorAttachmentViewStatus.PENDING:
            sensor_id = state.pending_sensor_id
            sensor_name = self._sensor_display_name(sensor_id)
        elif state.status is SensorAttachmentViewStatus.NONE:
            sensor_id = ""
            sensor_name = "No sensor"
        else:
            return None

        answer = QMessageBox.question(
            self,
            "Confirm sensor state",
            f'Confirm sensor state "{sensor_name}"?',
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if answer != QMessageBox.Yes:
            return None
        return self._confirm_sensor_attachment(
            sensor_id,
            state.attachment_revision,
        )

    def _create_sensor_definition(self, intent):
        return self._save_sensor_definition(intent, update=False)

    def _update_sensor_definition(self, intent):
        return self._save_sensor_definition(intent, update=True)

    def _save_sensor_definition(self, intent, update):
        if self.sensor_registry_client is None:
            self.sensor_controls.finish_sensor_save(
                False,
                "ROS is unavailable",
            )
            return None
        operation = (
            self.sensor_registry_client.update_sensor
            if update
            else self.sensor_registry_client.create_sensor
        )
        try:
            future = operation(
                intent.sensor_id,
                intent.display_name,
                intent.translation_m,
                intent.rotation_degrees,
                intent.channels,
            )
        except (TypeError, ValueError) as exception:
            self.sensor_controls.finish_sensor_save(
                False,
                str(exception),
            )
            return None
        if future is not None:
            self.sensor_controls.mark_sensor_save_pending()
            self.status_label.setText(
                "Status: saving sensor transform"
            )
        return future

    def _process_sensor_save_result(self, success, message):
        if hasattr(self, "sensor_controls"):
            self.sensor_controls.finish_sensor_save(
                bool(success),
                message,
            )
        if success:
            self.status_label.setText(f"Status: {message}")
            return
        self._process_application_error(message)

    def _delete_sensor_definition(self, sensor_id):
        if self.sensor_registry_client is None:
            self._process_application_error("ROS is unavailable")
            return None
        definition = self._sensor_definitions.get(sensor_id)
        display_name = (
            definition.display_name
            if definition is not None
            else sensor_id
        )
        answer = QMessageBox.question(
            self,
            "Delete sensor mount",
            f'Delete sensor mount "{display_name}"?',
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if answer != QMessageBox.Yes:
            return None
        future = self.sensor_registry_client.delete_sensor(sensor_id)
        if future is not None:
            self.status_label.setText(
                f"Status: deleting sensor {sensor_id}"
            )
        return future

    def _process_sensor_deletion_result(
        self,
        sensor_id,
        success,
        message,
    ):
        if hasattr(self, "sensor_controls"):
            self.sensor_controls.finish_sensor_deletion(
                sensor_id,
                bool(success),
                message,
            )
        if success:
            self.status_label.setText(f"Status: {message}")
            return
        self._process_application_error(message)

    def _select_sensor_attachment(self, sensor_id):
        if self.sensor_attachment_client is None:
            self._process_application_error("ROS is unavailable")
            return None
        future = self.sensor_attachment_client.select(sensor_id)
        if future is not None:
            self.status_label.setText("Sensor selection requested")
        return future

    def _confirm_sensor_attachment(self, sensor_id, attachment_revision):
        if self.sensor_attachment_client is None:
            self._process_application_error("ROS is unavailable")
            return None
        future = self.sensor_attachment_client.confirm(
            sensor_id,
            attachment_revision,
        )
        if future is not None:
            self.status_label.setText("Sensor attachment confirmation requested")
        return future

    def _on_tab_changed(self, index):
        if self.tabs.tabText(index) == "Navigation Control":
            self.navigation_controls._apply_map_list()
            self.navigation_controls._apply_waypoint_list()
        if self.tabs.tabText(index) == "Inspection Control":
            self.inspection_controls.refresh_setup_state()

    def _inspection_root_parameter(self):
        if self.node is None:
            return None
        parameter_name = "inspection.object_root"
        if not self.node.has_parameter(parameter_name):
            self.node.declare_parameter(parameter_name, "")
        value = self.node.get_parameter(parameter_name).value
        return value.strip() or None

    def _measurement_root_parameter(self) -> Path:
        default_root = default_measurement_root()
        if self.node is None:
            return default_root
        parameter_name = "measurement.root"
        if not self.node.has_parameter(parameter_name):
            self.node.declare_parameter(parameter_name, str(default_root))
        configured_root = str(
            self.node.get_parameter(parameter_name).value
        ).strip()
        return Path(configured_root or default_root).expanduser()

    def open_measurement_directory(self) -> bool:
        """Open the configured sensor measurement root in the file manager."""
        try:
            self.measurement_root.mkdir(parents=True, exist_ok=True)
        except OSError as exception:
            self._process_application_error(str(exception))
            return False
        opened = QDesktopServices.openUrl(
            QUrl.fromLocalFile(str(self.measurement_root))
        )
        if not opened:
            self._process_application_error(
                f"Could not open {self.measurement_root}"
            )
        return bool(opened)

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

    def add_sensor_control_tab(self):
        self.tabs.addTab(self.sensor_controls, "Sensor Mounts")

    def init_ros_communication(self):
        self.application_client = ApplicationClient(self.node)
        self.micro_ros_agent_status_client = MicroRosAgentStatusClient(
            self.node
        )
        self.micro_ros_agent_status_client.state_changed.connect(
            self._process_micro_ros_agent_status
        )
        self.sensor_registry_client = SensorRegistryClient(self.node)
        self.sensor_registry_client.definitions_changed.connect(
            self._process_sensor_definitions
        )
        self.sensor_registry_client.creation_finished.connect(
            self._process_sensor_save_result
        )
        self.sensor_registry_client.update_finished.connect(
            self._process_sensor_save_result
        )
        self.sensor_registry_client.deletion_finished.connect(
            self._process_sensor_deletion_result
        )
        self.sensor_topic_suggestion_client = (
            SensorTopicSuggestionClient(self.node)
        )
        self.sensor_topic_suggestion_client.suggestions_changed.connect(
            self._process_sensor_topic_suggestions
        )
        self.sensor_topic_suggestion_client.poll()
        self.sensor_attachment_client = SensorAttachmentClient(self.node)
        self.sensor_attachment_client.state_changed.connect(
            self._process_sensor_attachment_state
        )
        self.sensor_attachment_client.request_rejected.connect(
            self._process_application_error
        )
        self.sensor_head_connection_client = SensorHeadConnectionClient(
            self.node
        )
        self.sensor_head_connection_client.state_changed.connect(
            self._process_sensor_head_connection_state
        )
        self.sensor_acquisition_client = SensorAcquisitionClient(self.node)
        self.sensor_acquisition_client.state_changed.connect(
            self._process_sensor_acquisition_state
        )
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
        self.probe_setup_client = ProbeSetupClient(
            self.node,
            self.application_client.client_id,
        )
        self.probe_setup_client.state_changed.connect(
            self._process_probe_setup_state
        )
        self.probe_setup_client.request_rejected.connect(
            self._process_application_error
        )
        self.probe_setup_client.preview_received.connect(
            self._process_probe_reference_preview
        )
        self.probe_setup_client.preview_rejected.connect(
            self._process_probe_reference_preview_error
        )

        self.visible_tags_sub = self.node.create_subscription(
            TagElementArray,
            "fault_detector/state/visible_tags",
            self._process_visible_tags,
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
        client = self.micro_ros_agent_status_client
        if (
            client is not None
            and client.is_stale()
            and not self._agent_status_stale
        ):
            self._agent_status_stale = True
            last_state = client.last_state
            self.set_micro_ros_agent_status(
                None,
                getattr(last_state, "advertised_address", ""),
                getattr(last_state, "port", 0),
                "Agent status updates stopped",
            )
        sensor_client = self.sensor_head_connection_client
        if (
            sensor_client is not None
            and sensor_client.is_stale()
            and not self._sensor_connection_stale
        ):
            self._sensor_connection_stale = True
            self._sensor_head_connection_state = None
            self.set_sensor_connection_status(
                "unknown",
                detail="Sensor-head status updates stopped",
            )
            if hasattr(self, "sensor_controls"):
                self.sensor_controls.apply_sensor_head_connection(None)
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
        self._show_local_command_failure(state, intent_name)
        if hasattr(self, "inspection_controls"):
            self.inspection_controls.handle_application_state(state)

    def _show_local_command_failure(self, state, intent_name):
        if state.state != ApplicationCommandState.STATE_FAILED:
            return
        client_id = getattr(self.application_client, "client_id", "")
        if not client_id or state.client_id != client_id:
            return
        title = intent_name.replace("_", " ").title()
        QMessageBox.warning(
            self,
            f"{title} failed",
            state.detail or "The command failed without further detail.",
        )

    def _process_application_error(self, detail):
        self.status_label.setText(f"Operation rejected: {detail}")

    def _process_sensor_acquisition_state(self, state):
        self.set_sensor_acquisition_state(state)

    def _toggle_sensor_recording(self):
        state = self._sensor_acquisition_state
        if state is None:
            return None
        return self._request_sensor_recording(
            state.status is not SensorAcquisitionViewStatus.RECORDING
        )

    def _request_sensor_recording(self, start):
        if self.sensor_acquisition_client is None:
            self._process_application_error("ROS is unavailable")
            return None
        intent = self.sensor_acquisition_client.make_intent(start)
        return self.execute_operation(intent)

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

    def _open_probe_setup(self):
        if self.probe_setup_client is None:
            return
        if self.probe_setup_client.context_id:
            self.probe_setup_timer.stop()
            return
        if self.probe_setup_client.open() is not None:
            self.probe_setup_timer.stop()

    def _process_probe_setup_state(self, state):
        if hasattr(self, "inspection_controls"):
            self.inspection_controls.apply_setup_state(state)
        self.status_label.setText(state.detail)

    def _process_probe_reference_preview(self, response):
        if hasattr(self, "inspection_controls"):
            self.inspection_controls.apply_reference_preview(response)

    def _process_probe_reference_preview_error(self, view_id, detail):
        if hasattr(self, "inspection_controls"):
            self.inspection_controls.apply_reference_preview_error(
                view_id,
                detail,
            )
        self.status_label.setText(detail)

    def execute_probe_setup(self, intent):
        if self.probe_setup_client is None:
            self._process_application_error("ROS is unavailable")
            return None
        request_id = self.probe_setup_client.execute(intent)
        if request_id is not None:
            self.status_label.setText("Probe setup request submitted")
        return request_id

    def execute_probe_reference_capture(
        self,
        reference_camera_ids,
        replace_existing=False,
    ):
        if self.probe_setup_client is None:
            self._process_application_error("ROS is unavailable")
            return None
        request_id = self.probe_setup_client.capture_reference_views(
            reference_camera_ids,
            replace_existing=replace_existing,
        )
        if request_id is not None:
            self.status_label.setText(
                "Reference dataset capture submitted"
            )
        return request_id

    def execute_move_close_to_surface(self):
        if self.application_client is None:
            self._process_application_error("ROS is unavailable")
            return None
        controls = getattr(self, "inspection_controls", None)
        presentation = getattr(
            controls,
            "_refinement_presentation",
            None,
        )
        if presentation is None:
            self._process_application_error(
                "Probe refinement target distance is unavailable"
            )
            return None
        intent = OperationalIntent()
        intent.intent = OperationalIntent.INTENT_MOVE_CLOSE_TO_SURFACE
        intent.target_surface_distance_m = float(
            presentation.target_surface_distance_m
        )
        request_id = self.execute_operation(intent)
        if request_id is not None:
            self.status_label.setText("Move close to surface submitted")
        return request_id

    def execute_probe_refinement_finalization(self, **kwargs):
        if self.probe_setup_client is None:
            self._process_application_error("ROS is unavailable")
            return None
        request_id = self.probe_setup_client.finalize_refinement(**kwargs)
        if request_id is not None:
            self.status_label.setText(
                "Probe refinement finalization submitted"
            )
        return request_id

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
        self.probe_setup_timer.stop()
        if self.probe_setup_client is not None:
            self.probe_setup_client.close()
            self.probe_setup_client.destroy()
        if self.navigation_setup_client is not None:
            self.navigation_setup_client.close()
            self.navigation_setup_client.destroy()
        if self.sensor_registry_client is not None:
            self.sensor_registry_client.destroy()
        if self.sensor_topic_suggestion_client is not None:
            self.sensor_topic_suggestion_client.destroy()
        if self.sensor_attachment_client is not None:
            self.sensor_attachment_client.destroy()
        if self.sensor_head_connection_client is not None:
            self.sensor_head_connection_client.destroy()
        if self.sensor_acquisition_client is not None:
            self.sensor_acquisition_client.destroy()
        if self.micro_ros_agent_status_client is not None:
            self.micro_ros_agent_status_client.destroy()
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
