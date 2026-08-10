"""Inspection setup controls."""

import json
import math
import time
from collections import deque
from copy import deepcopy

from bosdyn.client.frame_helpers import HAND_FRAME_NAME
from PyQt5.QtCore import QLocale, Qt, QTimer
from PyQt5.QtGui import QColor, QDoubleValidator, QIntValidator
from PyQt5.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QFormLayout,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSplitter,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from fault_detector_msgs.msg import (
    CommandStatus,
    ComplexCommand,
    SensorDefinitionArray,
    TagElement,
)
from fault_detector_msgs.srv import AddSensor, RetireSensor
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
import tf2_ros

from fault_detector_spot.behaviour_tree.QOS_PROFILES import LATCHED_QOS
from fault_detector_spot.request_identity import new_request_id
from fault_detector_spot.inspection.models import (
    ImagePoint,
    InspectionObject,
    InspectionRoutine,
    MINIMUM_ALIGNED_PREAPPROACH_SEPARATION_M,
    PoseData,
    ProbePoint,
    QuaternionData,
    ReferenceTag,
    Vector3Data,
)
from fault_detector_spot.inspection.multi_reference_view_repository import (
    CapturedReferenceView,
    MultiReferenceViewRepository,
)
from fault_detector_spot.inspection.live_surface_distance import (
    aggregate_surface_distance_samples,
    measure_probe_surface_distance,
)
from fault_detector_spot.inspection.probe_refinement_session import (
    PendingRefinementMotion,
    ProbeRefinementSession,
    RefinementMotionState,
    RefinementStage,
)
from fault_detector_spot.inspection.reference_camera_registry import (
    REFERENCE_CAMERAS,
    REFERENCE_CAMERA_BY_ID,
)
from fault_detector_spot.inspection.reference_view_approach_direction import (
    APPROACH_MODE_AUTOMATIC,
    APPROACH_MODE_SURFACE_FIT,
    APPROACH_MODE_TAG_X,
    APPROACH_SOURCE_SURFACE_FIT,
    APPROACH_SOURCE_TAG_X_SELECTED,
    resolve_reference_approach_direction,
)
from fault_detector_spot.inspection.reference_view_depth_projection import (
    project_reference_pixel,
    rgb_depth_selectable_region,
)
from fault_detector_spot.inspection.reference_view_surface_normal import (
    estimate_reference_surface_normal,
)
from fault_detector_spot.inspection.reference_probe_setup import (
    approve_probe_pose,
    approve_safe_approach_pose,
    approve_surface_alignment_pose,
    compose_poses,
    initialize_reference_probe_setup,
    probe_pose_to_hand_pose,
    refine_probe_pose,
    relative_pose,
)
from fault_detector_spot.inspection.reference_view_surface_target import (
    quaternion_to_rpy,
    resolve_reference_surface_target,
)
from fault_detector_spot.inspection.sensor_models import (
    SensorDefinition,
    sensor_definition_from_values,
    sensor_probe_frame,
)
from fault_detector_spot.inspection.stable_tag_pose import (
    TagPoseSample,
    stabilize_tag_pose,
)

from ..commands.command_ids import CommandID, OrientationModes, TagFrames
from .collapsible_section import CollapsibleSection
from .probe_refinement_dialog import ProbeRefinementDialog
from .UIControlHelper import UIControlHelper
from .reference_view_widget import ReferenceViewWidget


MAX_REFINEMENT_TRANSLATION_M = 0.05
MAX_REFINEMENT_ROTATION_DEG = 15.0
MAX_SURFACE_CORRECTION_STEP_M = 0.02
MAX_LIVE_DEPTH_AGE_SEC = 0.5
PROBE_MOTION_SETTLE_SEC = 0.5
SURFACE_DISTANCE_TOLERANCE_M = 0.005
SURFACE_DISTANCE_STABILITY_TOLERANCE_M = 0.005
SURFACE_DISTANCE_SAMPLE_WINDOW_SEC = 0.20
BASE_TAG_MAXIMUM_AGE_SEC = 1.5
BASE_TAG_STABILIZATION_HISTORY_SEC = 3.0
BASE_TAG_HISTORY_MAX_SAMPLES = 64
BASE_TAG_MINIMUM_SPAN_SEC = 0.10
REFINEMENT_FRAME_SENSOR = "sensor"
REFINEMENT_FRAME_HAND = "hand"
REFINEMENT_FRAME_TAG = "tag"
REFINEMENT_FRAME_BODY = "body"
REFINEMENT_FRAME_MAP = "map"


class InspectionControls(UIControlHelper):
    """Build and publish inspection-definition setup commands."""

    def __init__(self, parent_ui):
        """Create controls backed by the configured object repository."""
        object_root = getattr(
            parent_ui,
            "inspection_object_root",
            None,
        )
        self.reference_view_repository = MultiReferenceViewRepository(
            object_root
        )
        self.object_repository = (
            self.reference_view_repository.object_repository
        )
        self._selected_definition = None
        self._reference_rgb_size = None
        self._reference_depth_image = None
        self._reference_rgb_camera_info = None
        self._reference_camera_info = None
        self._reference_view = None
        self._reference_slot_captures = [None, None, None]
        self._active_reference_slot = None
        self._selected_surface_point = None
        self._selected_surface_normal = None
        self._surface_normal_error = ""
        self._selected_approach_direction = None
        self._selected_surface_target = None
        self._calculated_probe_setup = None
        self._probe_setup = None
        self._refinement_session = None
        self._tf_buffer = None
        self._tf_listener = None
        self._sensor_definitions = {}
        self._pending_sensor_selection = ""
        self._pending_sensor_retirement = ""
        self._latest_hand_depth_image = None
        self._latest_hand_depth_camera_info = None
        self._latest_hand_depth_received_monotonic = 0.0
        self._hand_depth_history = deque(maxlen=16)
        self._base_tag_histories = {}
        self._probe_motion_pending = False
        self._command_state = CommandStatus.STATE_IDLE
        self._buffered_command_count = 0
        self._last_command_completion_monotonic = 0.0
        self._refinement_workflow_active = False
        self._distance_failure_requires_retraction = False
        self._retraction_failed = False
        self._editing_probe_point_id = None
        self.sensor_add_client = None
        self.sensor_retire_client = None
        self.sensor_list_subscription = None
        self.hand_depth_subscription = None
        self.hand_depth_camera_info_subscription = None
        self.management_dialog = None
        super().__init__(parent_ui)
        initial_sensors = getattr(parent_ui, "sensor_definitions", [])
        if initial_sensors:
            self.set_sensor_definitions(initial_sensors)
        self.refresh_saved_definitions()

    def add_rows(self, layout):
        """Add the rows constructed during initialization."""
        for row in self.rows:
            layout.addLayout(row)

    def init_ros_communication(self):
        """Use the UI publisher and create transient TF access."""
        self.complex_command_publisher = self.ui.complex_command_publisher
        if self.node is not None:
            self._tf_buffer = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(
                self._tf_buffer,
                self.node,
            )
            self.sensor_add_client = self.node.create_client(
                AddSensor,
                "fault_detector/add_sensor",
            )
            self.sensor_retire_client = self.node.create_client(
                RetireSensor,
                "fault_detector/retire_sensor",
            )
            self.sensor_list_subscription = (
                self.node.create_subscription(
                    SensorDefinitionArray,
                    "fault_detector/sensors",
                    self._process_sensor_definitions,
                    LATCHED_QOS,
                )
            )
            self.hand_depth_subscription = self.node.create_subscription(
                Image,
                "/depth_registered/hand/image",
                self._process_live_hand_depth,
                qos_profile_sensor_data,
            )
            self.hand_depth_camera_info_subscription = (
                self.node.create_subscription(
                    CameraInfo,
                    "/depth_registered/hand/camera_info",
                    self._process_live_hand_depth_camera_info,
                    qos_profile_sensor_data,
                )
            )

    def make_rows(self):
        """Create a compact inspection setup workspace."""
        self._make_management_dialog()
        self._create_reference_widgets()

        workspace = QVBoxLayout()
        workspace.setSpacing(6)
        workspace.addLayout(self._make_saved_definitions_row())
        workspace.addWidget(self._make_workspace_splitter(), 1)
        return [workspace]

    def _make_saved_definitions_row(self):
        row = QHBoxLayout()
        row.setSpacing(6)
        row.addWidget(QLabel("Object:"))

        self.saved_object_dropdown = QComboBox()
        self.saved_object_dropdown.setMinimumWidth(170)
        self.saved_object_dropdown.currentIndexChanged.connect(
            self._load_selected_object
        )
        row.addWidget(self.saved_object_dropdown)

        self.delete_object_button = QPushButton("Delete Object")
        self.delete_object_button.setEnabled(False)
        self.delete_object_button.setToolTip(
            "Delete the object selected above and all of its routines"
        )
        self.delete_object_button.clicked.connect(
            self.handle_delete_object
        )
        row.addWidget(self.delete_object_button)

        row.addWidget(QLabel("Routine:"))
        self.saved_routine_dropdown = QComboBox()
        self.saved_routine_dropdown.setMinimumWidth(170)
        self.saved_routine_dropdown.currentIndexChanged.connect(
            self._load_selected_routine
        )
        row.addWidget(self.saved_routine_dropdown)

        self.delete_routine_button = QPushButton("Delete Routine")
        self.delete_routine_button.setEnabled(False)
        self.delete_routine_button.setToolTip(
            "Delete the currently selected routine"
        )
        self.delete_routine_button.clicked.connect(
            self.handle_delete_routine
        )
        row.addWidget(self.delete_routine_button)

        self.refresh_definitions_button = QPushButton("Refresh")
        self.refresh_definitions_button.clicked.connect(
            self.refresh_saved_definitions
        )
        row.addWidget(self.refresh_definitions_button)

        self.manage_definitions_button = QPushButton(
            "Create Object or Routine"
        )
        self.manage_definitions_button.clicked.connect(
            self.show_management_dialog
        )
        row.addWidget(self.manage_definitions_button)
        row.addStretch()

        self.reference_view_status_label = QLabel(
            "Reference view: no routine selected"
        )
        self.reference_view_status_label.setTextInteractionFlags(
            Qt.TextSelectableByMouse
        )
        return row

    def _make_management_dialog(self):
        parent = self.ui if isinstance(self.ui, QWidget) else None
        self.management_dialog = QDialog(parent)
        self.management_dialog.setWindowTitle(
            "Create Inspection Objects and Routines"
        )
        self.management_dialog.setModal(False)
        self.management_dialog.resize(680, 680)

        dialog_layout = QVBoxLayout(self.management_dialog)

        object_group = QGroupBox("Create inspection object")
        object_layout = QFormLayout(object_group)
        self.object_id_field = QLineEdit()
        self.object_id_field.setPlaceholderText("Object ID")
        object_layout.addRow("Object ID:", self.object_id_field)

        self.object_display_name_field = QLineEdit()
        self.object_display_name_field.setPlaceholderText("Display name")
        object_layout.addRow(
            "Display name:",
            self.object_display_name_field,
        )

        self.reference_tag_id_field = QLineEdit()
        self.reference_tag_id_field.setPlaceholderText("Tag ID")
        self.reference_tag_id_field.setValidator(
            QIntValidator(0, 2147483647, self.reference_tag_id_field)
        )
        object_layout.addRow("Reference tag ID:", self.reference_tag_id_field)

        self.reference_tag_family_field = QLineEdit("36h11")
        self.reference_tag_family_field.setPlaceholderText("Tag family")
        object_layout.addRow(
            "Reference tag family:",
            self.reference_tag_family_field,
        )

        object_buttons = QHBoxLayout()
        self.create_object_button = QPushButton("Create Object")
        self.create_object_button.clicked.connect(self.handle_create_object)
        object_buttons.addWidget(self.create_object_button)
        object_buttons.addStretch()
        object_layout.addRow(object_buttons)
        dialog_layout.addWidget(object_group)

        sensor_group = QGroupBox("Manage hand-mounted sensors")
        sensor_layout = QFormLayout(sensor_group)
        self.new_sensor_id_field = QLineEdit()
        self.new_sensor_id_field.setPlaceholderText("Unique mounting ID")
        sensor_layout.addRow("Sensor ID:", self.new_sensor_id_field)

        self.new_sensor_display_name_field = QLineEdit()
        self.new_sensor_display_name_field.setPlaceholderText(
            "Display name"
        )
        sensor_layout.addRow(
            "Display name:",
            self.new_sensor_display_name_field,
        )

        translation_row = QHBoxLayout()
        self.sensor_translation_fields = []
        for axis in ("X", "Y", "Z"):
            translation_row.addWidget(QLabel(f"{axis}:"))
            field = QLineEdit("0.0")
            field.setFixedWidth(78)
            field.setValidator(self._signed_number_validator(field))
            self.sensor_translation_fields.append(field)
            translation_row.addWidget(field)
        translation_row.addStretch()
        sensor_layout.addRow("Hand to probe [m]:", translation_row)

        rotation_row = QHBoxLayout()
        self.sensor_rotation_fields = []
        for axis in ("Roll", "Pitch", "Yaw"):
            rotation_row.addWidget(QLabel(f"{axis}:"))
            field = QLineEdit("0.0")
            field.setFixedWidth(78)
            field.setValidator(self._signed_number_validator(field))
            self.sensor_rotation_fields.append(field)
            rotation_row.addWidget(field)
        rotation_row.addStretch()
        sensor_layout.addRow("Rotation [deg]:", rotation_row)

        sensor_buttons = QHBoxLayout()
        self.add_sensor_button = QPushButton("Add Sensor")
        self.add_sensor_button.clicked.connect(self.handle_add_sensor)
        sensor_buttons.addWidget(self.add_sensor_button)
        self.sensor_creation_status_label = QLabel(
            "Calibration is fixed after saving."
        )
        sensor_buttons.addWidget(self.sensor_creation_status_label, 1)
        sensor_layout.addRow(sensor_buttons)

        self.retire_sensor_dropdown = QComboBox()
        self.retire_sensor_dropdown.setMinimumWidth(260)
        self.retire_sensor_dropdown.currentIndexChanged.connect(
            self._handle_retire_sensor_changed
        )
        sensor_layout.addRow(
            "Registered sensor:",
            self.retire_sensor_dropdown,
        )

        retire_sensor_buttons = QHBoxLayout()
        self.retire_sensor_button = QPushButton("Retire Sensor")
        self.retire_sensor_button.setEnabled(False)
        self.retire_sensor_button.clicked.connect(
            self.handle_retire_sensor
        )
        retire_sensor_buttons.addWidget(self.retire_sensor_button)
        retire_sensor_buttons.addStretch()
        sensor_layout.addRow(retire_sensor_buttons)
        dialog_layout.addWidget(sensor_group)

        routine_group = QGroupBox("Create inspection routine")
        routine_layout = QFormLayout(routine_group)

        self.routine_parent_object_dropdown = QComboBox()
        self.routine_parent_object_dropdown.setMinimumWidth(260)
        routine_layout.addRow(
            "Existing object:",
            self.routine_parent_object_dropdown,
        )

        self.routine_id_field = QLineEdit()
        self.routine_id_field.setPlaceholderText("Routine ID")
        routine_layout.addRow("Routine ID:", self.routine_id_field)

        self.routine_display_name_field = QLineEdit()
        self.routine_display_name_field.setPlaceholderText("Display name")
        routine_layout.addRow(
            "Display name:",
            self.routine_display_name_field,
        )

        self.sensor_id_field = QComboBox()
        self.sensor_id_field.setMinimumWidth(260)
        self.sensor_id_field.currentIndexChanged.connect(
            self._handle_routine_sensor_changed
        )
        routine_layout.addRow("Sensor mounting:", self.sensor_id_field)

        self.probe_frame_value_label = QLabel("No sensor selected")
        self.probe_frame_value_label.setTextInteractionFlags(
            Qt.TextSelectableByMouse
        )
        routine_layout.addRow(
            "Derived probe frame:",
            self.probe_frame_value_label,
        )

        routine_buttons = QHBoxLayout()
        self.create_routine_button = QPushButton("Create Routine")
        self.create_routine_button.clicked.connect(
            self.handle_create_routine
        )
        routine_buttons.addWidget(self.create_routine_button)
        routine_buttons.addStretch()
        routine_layout.addRow(routine_buttons)
        dialog_layout.addWidget(routine_group)
        self._populate_sensor_dropdown()

        self.storage_path_label = QLabel(
            f"Storage: {self.object_repository.root_dir}"
        )
        self.storage_path_label.setTextInteractionFlags(
            Qt.TextSelectableByMouse
        )
        dialog_layout.addWidget(self.storage_path_label)

        self.management_status_label = QLabel(
            "Create and delete operations update this repository directly."
        )
        self.management_status_label.setWordWrap(True)
        dialog_layout.addWidget(self.management_status_label)

        close_buttons = QDialogButtonBox(QDialogButtonBox.Close)
        close_buttons.rejected.connect(self.management_dialog.hide)
        dialog_layout.addWidget(close_buttons)

    def show_management_dialog(self):
        """Show clean creation forms with the current object selected."""
        selected_object_id = (
            self.saved_object_dropdown.currentData() or ""
        )
        self._refresh_routine_parent_objects(
            desired_object_id=selected_object_id,
        )
        self.object_id_field.clear()
        self.object_display_name_field.clear()
        self.reference_tag_id_field.clear()
        self.reference_tag_family_field.setText("36h11")
        self.routine_id_field.clear()
        self.routine_display_name_field.clear()
        self.new_sensor_id_field.clear()
        self.new_sensor_display_name_field.clear()
        for field in (
            *self.sensor_translation_fields,
            *self.sensor_rotation_fields,
        ):
            field.setText("0.0")
        self.sensor_creation_status_label.setText(
            "Calibration is fixed after saving."
        )
        self.management_status_label.setText(
            "Create a new object, or select an existing object "
            "for the new routine."
        )
        self.management_dialog.show()
        self.management_dialog.raise_()
        self.management_dialog.activateWindow()

    def _process_sensor_definitions(self, message):
        definitions = []
        for sensor_message in message.sensors:
            definition = SensorDefinition(
                sensor_id=sensor_message.sensor_id,
                display_name=sensor_message.display_name,
                hand_to_probe=PoseData(
                    position=Vector3Data(
                        x=sensor_message.hand_to_probe.position.x,
                        y=sensor_message.hand_to_probe.position.y,
                        z=sensor_message.hand_to_probe.position.z,
                    ),
                    orientation=QuaternionData(
                        x=sensor_message.hand_to_probe.orientation.x,
                        y=sensor_message.hand_to_probe.orientation.y,
                        z=sensor_message.hand_to_probe.orientation.z,
                        w=sensor_message.hand_to_probe.orientation.w,
                    ),
                ),
            )
            try:
                definition.validate()
            except Exception as exception:
                self._set_status_text(
                    "Ignored invalid sensor definition "
                    f"'{definition.sensor_id}': {exception}"
                )
                continue
            definitions.append(definition)
        self.set_sensor_definitions(definitions)

    def _process_live_hand_depth(self, message):
        self._latest_hand_depth_image = message
        self._latest_hand_depth_received_monotonic = time.monotonic()
        stamp = message.header.stamp
        stamp_key = (int(stamp.sec), int(stamp.nanosec))
        if (
            not self._hand_depth_history
            or self._hand_depth_history[-1][0] != stamp_key
        ):
            self._hand_depth_history.append(
                (
                    stamp_key,
                    self._latest_hand_depth_received_monotonic,
                    message,
                )
            )

    def _process_live_hand_depth_camera_info(self, message):
        self._latest_hand_depth_camera_info = message

    def handle_base_tags(self, tags):
        """Buffer distinct authoritative base-camera tag observations."""
        for tag in tags:
            stamp = tag.pose.header.stamp
            stamp_key = (int(stamp.sec), int(stamp.nanosec))
            history = self._base_tag_histories.setdefault(
                int(tag.id),
                deque(maxlen=BASE_TAG_HISTORY_MAX_SAMPLES),
            )
            if history and history[-1][0] == stamp_key:
                continue
            history.append((stamp_key, deepcopy(tag)))

    def handle_command_status(self, status):
        self._command_state = status.state
        self._buffered_command_count = status.buffered_command_count
        session = self._refinement_session
        pending = session.pending_motion if session is not None else None
        if status.state == CommandStatus.STATE_RUNNING:
            if pending is None and session is not None:
                self._clear_refinement_execution_evidence(
                    "An unrelated robot command started"
                )
            elif pending is not None and status.request_id != pending.request_id:
                self._fail_pending_refinement_motion(
                    "An unrelated robot command started during probe "
                    "refinement",
                    pending.request_id,
                )
            self._probe_motion_pending = (
                session is not None and session.pending_motion is not None
            )
            self._refresh_refinement_dialog()
            return

        if pending is None:
            self._probe_motion_pending = False
            return
        if status.request_id != pending.request_id:
            return
        if status.command_id != pending.command_id:
            self._fail_pending_refinement_motion(
                "Unexpected terminal command status for "
                f"{status.command_id}",
                status.request_id,
            )
            return
        self._last_command_completion_monotonic = time.monotonic()
        if status.state == CommandStatus.STATE_SUCCEEDED:
            QTimer.singleShot(
                int(PROBE_MOTION_SETTLE_SEC * 1000),
                lambda: self._complete_pending_refinement_motion(
                    status.request_id
                ),
            )
            return
        self._fail_pending_refinement_motion(
            f"Robot movement ended with state {status.state}: "
            f"{status.detail}",
            status.request_id,
        )

    def set_sensor_definitions(self, definitions):
        """Replace the UI sensor list with validated registry data."""
        desired_retired_sensor_id = ""
        if hasattr(self, "retire_sensor_dropdown"):
            desired_retired_sensor_id = (
                self.retire_sensor_dropdown.currentData() or ""
            )
        validated = {}
        for definition in definitions:
            definition.validate()
            if definition.sensor_id in validated:
                raise ValueError(
                    f"Duplicate sensor ID: {definition.sensor_id}"
                )
            validated[definition.sensor_id] = definition
        self._sensor_definitions = validated
        desired_sensor_id = (
            self._pending_sensor_selection
            or self.sensor_id_field.currentData()
            or ""
        )
        self._populate_sensor_dropdown(desired_sensor_id)
        if (
            self._pending_sensor_selection
            and self._pending_sensor_selection in validated
        ):
            self._pending_sensor_selection = ""
        self._populate_retire_sensor_dropdown(
            desired_retired_sensor_id
        )

    def _populate_sensor_dropdown(self, desired_sensor_id=""):
        if not hasattr(self, "sensor_id_field"):
            return
        dropdown = self.sensor_id_field
        dropdown.blockSignals(True)
        dropdown.clear()
        dropdown.addItem("Select registered sensor", None)
        for sensor_id in sorted(self._sensor_definitions):
            definition = self._sensor_definitions[sensor_id]
            dropdown.addItem(
                f"{definition.display_name} [{sensor_id}]",
                sensor_id,
            )
        selected_index = dropdown.findData(desired_sensor_id)
        if desired_sensor_id and selected_index < 0:
            dropdown.addItem(
                f"Unavailable sensor [{desired_sensor_id}]",
                desired_sensor_id,
            )
            selected_index = dropdown.count() - 1
            dropdown.setItemData(
                selected_index,
                QColor("red"),
                Qt.ForegroundRole,
            )
        dropdown.setCurrentIndex(
            selected_index if selected_index >= 0 else 0
        )
        dropdown.blockSignals(False)
        self._handle_routine_sensor_changed()

    def _populate_retire_sensor_dropdown(self, desired_sensor_id=""):
        if not hasattr(self, "retire_sensor_dropdown"):
            return
        dropdown = self.retire_sensor_dropdown
        dropdown.blockSignals(True)
        dropdown.clear()
        dropdown.addItem("Select registered sensor", None)
        for sensor_id in sorted(self._sensor_definitions):
            definition = self._sensor_definitions[sensor_id]
            dropdown.addItem(
                f"{definition.display_name} [{sensor_id}]",
                sensor_id,
            )
        selected_index = dropdown.findData(desired_sensor_id)
        dropdown.setCurrentIndex(
            selected_index if selected_index >= 0 else 0
        )
        dropdown.blockSignals(False)
        self._handle_retire_sensor_changed()

    def _handle_retire_sensor_changed(self, _index=None):
        sensor_id = self.retire_sensor_dropdown.currentData()
        self.retire_sensor_button.setEnabled(
            sensor_id in self._sensor_definitions
        )

    def _handle_routine_sensor_changed(self, _index=None):
        sensor_id = self.sensor_id_field.currentData()
        if not sensor_id:
            self.probe_frame_value_label.setText("No sensor selected")
        elif sensor_id not in self._sensor_definitions:
            self.probe_frame_value_label.setText(
                f"{sensor_probe_frame(sensor_id)} (sensor unavailable)"
            )
        else:
            self.probe_frame_value_label.setText(
                sensor_probe_frame(sensor_id)
            )
        if hasattr(self, "create_routine_button"):
            has_parent = bool(
                self.routine_parent_object_dropdown.currentData()
            )
            self.create_routine_button.setEnabled(
                has_parent and sensor_id in self._sensor_definitions
            )

    def handle_add_sensor(self):
        """Submit one immutable sensor mounting to the registry."""
        sensor_id = self._required_text(
            self.new_sensor_id_field,
            "a unique sensor ID",
        )
        display_name = self._required_text(
            self.new_sensor_display_name_field,
            "a sensor display name",
        )
        if sensor_id is None or display_name is None:
            return False
        try:
            translation = [
                self._signed_number_value(field, axis)
                for field, axis in zip(
                    self.sensor_translation_fields,
                    ("Translation X", "Translation Y", "Translation Z"),
                )
            ]
            rotation = [
                self._signed_number_value(field, axis)
                for field, axis in zip(
                    self.sensor_rotation_fields,
                    ("Roll", "Pitch", "Yaw"),
                )
            ]
            definition = sensor_definition_from_values(
                sensor_id,
                display_name,
                *translation,
                *rotation,
            )
        except Exception as exception:
            self.sensor_creation_status_label.setText(str(exception))
            self.show_warning("Add Sensor", str(exception))
            return False

        if self.sensor_add_client is None:
            message = "Sensor registry is unavailable"
            self.sensor_creation_status_label.setText(message)
            self.show_warning("Add Sensor", message)
            return False
        if not self.sensor_add_client.service_is_ready():
            message = "Sensor registry service is not ready"
            self.sensor_creation_status_label.setText(message)
            self.show_warning("Add Sensor", message)
            return False

        request = AddSensor.Request()
        request.sensor.sensor_id = definition.sensor_id
        request.sensor.display_name = definition.display_name
        request.sensor.hand_to_probe.position.x = (
            definition.hand_to_probe.position.x
        )
        request.sensor.hand_to_probe.position.y = (
            definition.hand_to_probe.position.y
        )
        request.sensor.hand_to_probe.position.z = (
            definition.hand_to_probe.position.z
        )
        request.sensor.hand_to_probe.orientation.x = (
            definition.hand_to_probe.orientation.x
        )
        request.sensor.hand_to_probe.orientation.y = (
            definition.hand_to_probe.orientation.y
        )
        request.sensor.hand_to_probe.orientation.z = (
            definition.hand_to_probe.orientation.z
        )
        request.sensor.hand_to_probe.orientation.w = (
            definition.hand_to_probe.orientation.w
        )
        self._pending_sensor_selection = definition.sensor_id
        self.add_sensor_button.setEnabled(False)
        self.sensor_creation_status_label.setText(
            f"Adding sensor '{definition.sensor_id}'..."
        )
        future = self.sensor_add_client.call_async(request)
        future.add_done_callback(self._handle_add_sensor_response)
        return True

    def _handle_add_sensor_response(self, future):
        self.add_sensor_button.setEnabled(True)
        try:
            response = future.result()
        except Exception as exception:
            self._pending_sensor_selection = ""
            self.sensor_creation_status_label.setText(
                f"Sensor creation failed: {exception}"
            )
            return
        self.sensor_creation_status_label.setText(response.message)
        if not response.success:
            self._pending_sensor_selection = ""
            return
        self._set_status_text(response.message)
        self.new_sensor_id_field.clear()
        self.new_sensor_display_name_field.clear()

    def handle_retire_sensor(self):
        """Retire one unreferenced sensor mounting through the registry."""
        sensor_id = self.retire_sensor_dropdown.currentData()
        if sensor_id not in self._sensor_definitions:
            self.show_warning(
                "Retire Sensor",
                "Select a registered sensor mounting.",
            )
            return False
        if not self.ask_question(
            "Retire Sensor",
            (
                f"Retire sensor '{sensor_id}'? Its ID cannot be reused. "
                "The complete system must be restarted to clear its "
                "static TF."
            ),
        ):
            return False
        if self.sensor_retire_client is None:
            message = "Sensor registry is unavailable"
            self.sensor_creation_status_label.setText(message)
            self.show_warning("Retire Sensor", message)
            return False
        if not self.sensor_retire_client.service_is_ready():
            message = "Sensor registry service is not ready"
            self.sensor_creation_status_label.setText(message)
            self.show_warning("Retire Sensor", message)
            return False

        request = RetireSensor.Request()
        request.sensor_id = sensor_id
        self._pending_sensor_retirement = sensor_id
        self.retire_sensor_button.setEnabled(False)
        self.create_routine_button.setEnabled(False)
        self.sensor_creation_status_label.setText(
            f"Retiring sensor '{sensor_id}'..."
        )
        future = self.sensor_retire_client.call_async(request)
        future.add_done_callback(self._handle_retire_sensor_response)
        return True

    def _handle_retire_sensor_response(self, future):
        try:
            response = future.result()
        except Exception as exception:
            self._pending_sensor_retirement = ""
            self._handle_retire_sensor_changed()
            self._handle_routine_sensor_changed()
            self.sensor_creation_status_label.setText(
                f"Sensor retirement failed: {exception}"
            )
            return
        self.sensor_creation_status_label.setText(response.message)
        if not response.success:
            self._pending_sensor_retirement = ""
            self._handle_retire_sensor_changed()
            self._handle_routine_sensor_changed()
            return
        retired_sensor_id = self._pending_sensor_retirement
        self._pending_sensor_retirement = ""
        self._sensor_definitions.pop(retired_sensor_id, None)
        self._populate_sensor_dropdown()
        self._populate_retire_sensor_dropdown()
        self._set_status_text(response.message)

    @staticmethod
    def _signed_number_value(field, label):
        text = field.text().strip()
        try:
            value = float(text)
        except ValueError as exception:
            raise ValueError(f"{label} must be a number") from exception
        if not math.isfinite(value):
            raise ValueError(f"{label} must be finite")
        return value

    @staticmethod
    def _signed_number_validator(parent):
        validator = QDoubleValidator(-1000.0, 1000.0, 9, parent)
        validator.setNotation(QDoubleValidator.StandardNotation)
        validator.setLocale(QLocale.c())
        return validator

    def _create_reference_widgets(self):
        self.reference_view_widgets = [
            ReferenceViewWidget(),
            ReferenceViewWidget(),
            ReferenceViewWidget(),
        ]
        self.reference_view_widget = self.reference_view_widgets[0]
        self.reference_camera_dropdowns = []
        self.replace_reference_view_checkbox = QCheckBox(
            "Replace existing"
        )
        self.capture_reference_view_button = QPushButton(
            "Capture Reference View"
        )
        self.capture_reference_view_button.clicked.connect(
            self.handle_capture_reference_view
        )

        self.reference_pixel_value_label = self._fixed_readout_label(
            "—",
            130,
        )
        self.clear_reference_pixel_button = QPushButton("Clear Point")
        self.clear_reference_pixel_button.setEnabled(False)
        self.clear_reference_pixel_button.clicked.connect(
            self._clear_all_reference_selections
        )

        self.reference_surface_frame_value_label = (
            self._fixed_readout_label("—", 210)
        )
        self.reference_surface_x_value_label = self._fixed_readout_label(
            "—",
            80,
        )
        self.reference_surface_y_value_label = self._fixed_readout_label(
            "—",
            80,
        )
        self.reference_surface_z_value_label = self._fixed_readout_label(
            "—",
            80,
        )
        self.reference_depth_pixel_value_label = (
            self._fixed_readout_label("—", 145)
        )
        self.reference_projection_status_label = (
            self._fixed_readout_label("No point", 155)
        )
        self.reference_normal_x_value_label = self._fixed_readout_label(
            "—",
            80,
        )
        self.reference_normal_y_value_label = self._fixed_readout_label(
            "—",
            80,
        )
        self.reference_normal_z_value_label = self._fixed_readout_label(
            "—",
            80,
        )
        self.reference_normal_samples_value_label = (
            self._fixed_readout_label("—", 70)
        )
        self.reference_normal_rmse_value_label = (
            self._fixed_readout_label("—", 90)
        )
        self.reference_normal_status_label = (
            self._fixed_readout_label("No point", 155)
        )

        self.reference_approach_mode_dropdown = QComboBox()
        self.reference_approach_mode_dropdown.addItem(
            "Automatic surface fit",
            APPROACH_MODE_AUTOMATIC,
        )
        self.reference_approach_mode_dropdown.addItem(
            "Surface fit only",
            APPROACH_MODE_SURFACE_FIT,
        )
        self.reference_approach_mode_dropdown.addItem(
            "Tag +X (calibrated mount only)",
            APPROACH_MODE_TAG_X,
        )
        self.reference_approach_source_value_label = (
            self._fixed_readout_label("—", 155)
        )
        self.reference_approach_status_label = (
            self._fixed_readout_label("No point", 155)
        )

        self.reference_target_distance_field = QLineEdit("0.03")
        self.reference_target_distance_field.setFixedWidth(90)
        self.reference_target_distance_field.setValidator(
            self._distance_validator(self.reference_target_distance_field)
        )
        self.reference_preapproach_distance_field = QLineEdit("0.10")
        self.reference_preapproach_distance_field.setFixedWidth(90)
        self.reference_preapproach_distance_field.setValidator(
            self._distance_validator(
                self.reference_preapproach_distance_field
            )
        )
        self.reference_target_status_label = self._fixed_readout_label(
            "No point",
            155,
        )
        self.reference_target_x_value_label = self._fixed_readout_label(
            "—",
            80,
        )
        self.reference_target_y_value_label = self._fixed_readout_label(
            "—",
            80,
        )
        self.reference_target_z_value_label = self._fixed_readout_label(
            "—",
            80,
        )
        self.reference_target_roll_value_label = self._fixed_readout_label(
            "—",
            80,
        )
        self.reference_target_pitch_value_label = self._fixed_readout_label(
            "—",
            80,
        )
        self.reference_target_yaw_value_label = self._fixed_readout_label(
            "—",
            80,
        )
        self.reference_preapproach_x_value_label = (
            self._fixed_readout_label("—", 80)
        )
        self.reference_preapproach_y_value_label = (
            self._fixed_readout_label("—", 80)
        )
        self.reference_preapproach_z_value_label = (
            self._fixed_readout_label("—", 80)
        )

        self.reference_setup_status_label = self._fixed_readout_label(
            "No point",
            210,
        )
        self.move_calculated_approach_button = QPushButton(
            "Move to Candidate"
        )
        self.use_current_approach_button = QPushButton(
            "Approve Current Pose"
        )
        self.move_aligned_pose_button = QPushButton(
            "Move to Candidate"
        )
        self.use_current_alignment_button = QPushButton(
            "Approve Current Pose"
        )
        self.move_probe_pose_button = QPushButton(
            "Direct Probe Movement Disabled"
        )
        self.move_probe_pose_button.setEnabled(False)
        self.use_current_probe_button = QPushButton("Approve Probe Pose")
        self.use_current_probe_button.setEnabled(False)

        self.approach_step_status_label = QLabel("Waiting for target")
        self.alignment_step_status_label = QLabel("Waiting for target")
        self.probe_step_status_label = QLabel("Waiting for alignment")
        self.save_approach_status_label = QLabel("Not approved")
        self.save_alignment_status_label = QLabel("Not approved")
        self.save_probe_status_label = QLabel("Not approved")

        self.refine_translation_step_field = QLineEdit("0.01")
        self.refine_translation_step_field.setFixedWidth(70)
        self.refine_translation_step_field.setValidator(
            self._bounded_number_validator(
                self.refine_translation_step_field,
                0.001,
                MAX_REFINEMENT_TRANSLATION_M,
                3,
            )
        )
        self.refine_rotation_step_field = QLineEdit("2.0")
        self.refine_rotation_step_field.setFixedWidth(70)
        self.refine_rotation_step_field.setValidator(
            self._bounded_number_validator(
                self.refine_rotation_step_field,
                0.1,
                MAX_REFINEMENT_ROTATION_DEG,
                1,
            )
        )
        self.refine_frame_dropdown = QComboBox()
        for label, frame in (
            ("Sensor", REFINEMENT_FRAME_SENSOR),
            ("Hand", REFINEMENT_FRAME_HAND),
            ("Tag", REFINEMENT_FRAME_TAG),
            ("Body", REFINEMENT_FRAME_BODY),
            ("Map", REFINEMENT_FRAME_MAP),
        ):
            self.refine_frame_dropdown.addItem(label, frame)
        self.refine_frame_dropdown.setToolTip(
            "Coordinate frame used by each finite relative adjustment"
        )
        self.refinement_buttons = {}
        for stage in ("approach", "alignment", "probe"):
            actions = [
                "up",
                "down",
                "left",
                "right",
                "pitch_up",
                "pitch_down",
                "yaw_left",
                "yaw_right",
            ]
            if stage == "approach":
                actions.extend(("front", "back"))
            if stage == "probe":
                actions = []
            stage_buttons = {}
            for action in actions:
                button = QPushButton(
                    self._refinement_button_label(action)
                )
                button.clicked.connect(
                    lambda _checked=False, selected_stage=stage,
                    selected_action=action: self.handle_refine_pose(
                        selected_stage,
                        selected_action,
                    )
                )
                stage_buttons[action] = button
                setattr(
                    self,
                    f"{stage}_refine_{action}_button",
                    button,
                )
            self.refinement_buttons[stage] = stage_buttons

        self.live_surface_distance_value_label = (
            self._fixed_readout_label("—", 90)
        )
        self.surface_distance_delta_value_label = (
            self._fixed_readout_label("—", 90)
        )
        self.test_surface_distance_button = QPushButton(
            "Test Surface Distance"
        )
        self.test_surface_distance_button.setToolTip(
            "Measure fresh hand-camera depth and command one bounded "
            "correction along the probe axis"
        )
        self.surface_distance_test_status_label = QLabel(
            "Waiting for aligned pre-approach approval"
        )
        self.surface_distance_test_status_label.setWordWrap(True)
        self.surface_distance_tolerance_field = QLineEdit(
            f"{SURFACE_DISTANCE_TOLERANCE_M:.3f}"
        )
        self.surface_distance_tolerance_field.setFixedWidth(90)
        self.surface_distance_tolerance_field.setValidator(
            self._bounded_number_validator(
                self.surface_distance_tolerance_field,
                0.001,
                0.05,
                3,
            )
        )
        self.approve_and_retract_button = QPushButton(
            "Approve and Retract"
        )
        self.approve_and_retract_button.setEnabled(False)
        self.retract_without_saving_button = QPushButton(
            "Retract Without Saving"
        )
        self.retract_without_saving_button.setEnabled(False)
        self.refinement_recovery_status_label = QLabel("")
        self.refinement_recovery_status_label.setWordWrap(True)
        self.refinement_summary_status_label = QLabel("")
        self.refinement_summary_status_label.setWordWrap(True)
        self.start_probe_refinement_button = QPushButton(
            "Start Probe Point Position Refinement Workflow"
        )
        self.start_probe_refinement_button.setEnabled(False)

        self.probe_point_id_field = QLineEdit()
        self.probe_point_id_field.setPlaceholderText("Probe point ID")
        self.probe_point_display_name_field = QLineEdit()
        self.probe_point_display_name_field.setPlaceholderText(
            "Display name"
        )
        self.probe_position_tolerance_field = QLineEdit("0.01")
        self.probe_orientation_tolerance_field = QLineEdit("0.087")
        self.probe_measurement_duration_field = QLineEdit("1.0")
        self.save_probe_point_status_label = QLabel(
            "Approve all three poses and complete the definition."
        )
        self.save_probe_point_status_label.setWordWrap(True)

        self._set_probe_setup_buttons_enabled(False)
        for slot_index, widget in enumerate(self.reference_view_widgets):
            widget.image_point_changed.connect(
                lambda u, v, slot=slot_index:
                self._handle_reference_slot_point_changed(slot, u, v)
            )
            widget.image_point_cleared.connect(
                lambda slot=slot_index:
                self._handle_reference_slot_point_cleared(slot)
            )
        self.reference_approach_mode_dropdown.currentIndexChanged.connect(
            self._handle_approach_mode_changed
        )
        self.reference_target_distance_field.editingFinished.connect(
            self._handle_target_distance_changed
        )
        self.reference_preapproach_distance_field.editingFinished.connect(
            self._handle_target_distance_changed
        )
        self.move_calculated_approach_button.clicked.connect(
            self.handle_move_to_approach_pose
        )
        self.use_current_approach_button.clicked.connect(
            self.handle_use_current_as_approach
        )
        self.move_aligned_pose_button.clicked.connect(
            self.handle_move_to_aligned_pose
        )
        self.use_current_alignment_button.clicked.connect(
            self.handle_use_current_alignment
        )
        self.test_surface_distance_button.clicked.connect(
            self.handle_test_surface_distance
        )
        self.approve_and_retract_button.clicked.connect(
            self.handle_approve_and_retract
        )
        self.retract_without_saving_button.clicked.connect(
            self.handle_retract_without_saving
        )
        self.start_probe_refinement_button.clicked.connect(
            self.handle_start_probe_refinement
        )
        for field in (
            self.probe_point_id_field,
            self.probe_point_display_name_field,
            self.probe_position_tolerance_field,
            self.probe_orientation_tolerance_field,
            self.probe_measurement_duration_field,
        ):
            field.textChanged.connect(
                self._update_save_probe_point_state
            )
        self.surface_distance_tolerance_field.textChanged.connect(
            self._handle_surface_tolerance_changed
        )
        self.refinement_dialog = ProbeRefinementDialog(self)

    def _make_workspace_splitter(self):
        self.inspection_workspace_splitter = QSplitter(Qt.Vertical)
        self.inspection_workspace_splitter.setChildrenCollapsible(False)
        self.inspection_workspace_splitter.addWidget(
            self._make_reference_view_panel()
        )
        self.inspection_workspace_splitter.addWidget(
            self._make_workflow_tabs()
        )
        self.inspection_workspace_splitter.setStretchFactor(0, 2)
        self.inspection_workspace_splitter.setStretchFactor(1, 3)
        self.inspection_workspace_splitter.setSizes([320, 500])
        return self.inspection_workspace_splitter

    def _make_reference_view_panel(self):
        panel = QFrame()
        panel.setFrameShape(QFrame.StyledPanel)
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        toolbar = QHBoxLayout()
        toolbar.addWidget(QLabel("Reference view:"))
        toolbar.addWidget(self.reference_view_status_label, 1)
        toolbar.addWidget(self.replace_reference_view_checkbox)
        toolbar.addWidget(self.capture_reference_view_button)
        layout.addLayout(toolbar)

        camera_row = QHBoxLayout()
        camera_row.setSpacing(8)
        self.reference_camera_slots = []
        for slot_index, widget in enumerate(self.reference_view_widgets):
            widget.setMinimumSize(240, 180)
            slot = self._make_camera_slot(slot_index, widget)
            self.reference_camera_slots.append(slot)
            camera_row.addWidget(slot, 1)
        layout.addLayout(camera_row, 1)

        selection_row = QHBoxLayout()
        selection_row.addWidget(QLabel("Selected pixel:"))
        selection_row.addWidget(self.reference_pixel_value_label)
        selection_row.addWidget(self.clear_reference_pixel_button)
        selection_row.addStretch()
        layout.addLayout(selection_row)
        return panel

    def _make_camera_slot(self, slot_index, content):
        slot = QFrame()
        slot.setFrameShape(QFrame.StyledPanel)
        slot_layout = QVBoxLayout(slot)
        slot_layout.setContentsMargins(4, 4, 4, 4)
        slot_layout.setSpacing(4)

        selector_row = QHBoxLayout()
        selector_row.addWidget(QLabel(f"Camera {slot_index + 1}:"))
        dropdown = QComboBox()
        dropdown.addItem("None", "")
        for camera in REFERENCE_CAMERAS:
            dropdown.addItem(camera.display_name, camera.camera_id)
        default_id = "hand" if slot_index == 0 else ""
        dropdown.setCurrentIndex(dropdown.findData(default_id))
        dropdown.currentIndexChanged.connect(
            lambda _index, slot=slot_index:
            self._handle_reference_camera_selection_changed(slot)
        )
        self.reference_camera_dropdowns.append(dropdown)
        selector_row.addWidget(dropdown, 1)
        slot_layout.addLayout(selector_row)
        slot_layout.addWidget(content, 1)
        if not default_id:
            content.clear_preview("No camera selected")
        return slot

    def _make_workflow_tabs(self):
        self.workflow_tabs = QTabWidget()
        self.workflow_tabs.addTab(
            self._scrollable_tab(self._make_target_tab()),
            "Target",
        )
        self.workflow_tabs.addTab(
            self._scrollable_tab(self._make_refine_tab()),
            "Refine",
        )
        self.workflow_tabs.addTab(
            self._scrollable_tab(self._make_save_tab()),
            "Save",
        )
        return self.workflow_tabs

    @staticmethod
    def _scrollable_tab(content):
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setWidget(content)
        return scroll

    def _make_target_tab(self):
        self.reference_point_panel = QFrame()
        layout = QVBoxLayout(self.reference_point_panel)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        orientation_group = QGroupBox("Surface orientation")
        orientation_layout = QGridLayout(orientation_group)
        orientation_layout.addWidget(QLabel("Mode:"), 0, 0)
        orientation_layout.addWidget(
            self.reference_approach_mode_dropdown,
            0,
            1,
            1,
            2,
        )
        orientation_layout.addWidget(QLabel("Used:"), 1, 0)
        orientation_layout.addWidget(
            self.reference_approach_source_value_label,
            1,
            1,
        )
        orientation_layout.addWidget(QLabel("Status:"), 1, 2)
        orientation_layout.addWidget(
            self.reference_approach_status_label,
            1,
            3,
        )
        layout.addWidget(orientation_group)

        target_group = QGroupBox("Calculated poses")
        target_layout = QGridLayout(target_group)
        target_layout.addWidget(
            QLabel("Aligned pre-approach distance [m]:"),
            0,
            0,
        )
        target_layout.addWidget(
            self.reference_preapproach_distance_field,
            0,
            1,
        )
        target_layout.addWidget(QLabel("Status:"), 1, 0)
        target_layout.addWidget(
            self.reference_target_status_label,
            1,
            1,
            1,
            3,
        )

        target_layout.addWidget(QLabel("Target position [m]:"), 2, 0)
        target_layout.addWidget(QLabel("x"), 2, 1)
        target_layout.addWidget(self.reference_target_x_value_label, 2, 2)
        target_layout.addWidget(QLabel("y"), 2, 3)
        target_layout.addWidget(self.reference_target_y_value_label, 2, 4)
        target_layout.addWidget(QLabel("z"), 2, 5)
        target_layout.addWidget(self.reference_target_z_value_label, 2, 6)

        target_layout.addWidget(
            QLabel("Target angle relative to object [deg]:"),
            3,
            0,
        )
        target_layout.addWidget(QLabel("roll"), 3, 1)
        target_layout.addWidget(
            self.reference_target_roll_value_label,
            3,
            2,
        )
        target_layout.addWidget(QLabel("pitch"), 3, 3)
        target_layout.addWidget(
            self.reference_target_pitch_value_label,
            3,
            4,
        )
        target_layout.addWidget(QLabel("yaw"), 3, 5)
        target_layout.addWidget(
            self.reference_target_yaw_value_label,
            3,
            6,
        )

        target_layout.addWidget(
            QLabel("Aligned pre-approach position [m]:"),
            4,
            0,
        )
        target_layout.addWidget(QLabel("x"), 4, 1)
        target_layout.addWidget(
            self.reference_preapproach_x_value_label,
            4,
            2,
        )
        target_layout.addWidget(QLabel("y"), 4, 3)
        target_layout.addWidget(
            self.reference_preapproach_y_value_label,
            4,
            4,
        )
        target_layout.addWidget(QLabel("z"), 4, 5)
        target_layout.addWidget(
            self.reference_preapproach_z_value_label,
            4,
            6,
        )
        target_layout.setColumnStretch(7, 1)
        layout.addWidget(target_group)

        self.geometry_details_section = CollapsibleSection(
            "Geometry details",
            self._make_geometry_details_widget(),
            expanded=False,
        )
        layout.addWidget(self.geometry_details_section)
        layout.addStretch()
        return self.reference_point_panel

    def _make_geometry_details_widget(self):
        widget = QWidget()
        layout = QGridLayout(widget)
        layout.setContentsMargins(0, 0, 0, 0)

        layout.addWidget(QLabel("Frame:"), 0, 0)
        layout.addWidget(self.reference_surface_frame_value_label, 0, 1)
        layout.addWidget(QLabel("Projection:"), 0, 2)
        layout.addWidget(self.reference_projection_status_label, 0, 3)

        layout.addWidget(QLabel("Surface point [m]:"), 1, 0)
        layout.addWidget(QLabel("x"), 1, 1)
        layout.addWidget(self.reference_surface_x_value_label, 1, 2)
        layout.addWidget(QLabel("y"), 1, 3)
        layout.addWidget(self.reference_surface_y_value_label, 1, 4)
        layout.addWidget(QLabel("z"), 1, 5)
        layout.addWidget(self.reference_surface_z_value_label, 1, 6)

        layout.addWidget(QLabel("Depth source:"), 2, 0)
        layout.addWidget(self.reference_depth_pixel_value_label, 2, 1, 1, 2)
        layout.addWidget(QLabel("Normal status:"), 2, 3)
        layout.addWidget(self.reference_normal_status_label, 2, 4, 1, 2)

        layout.addWidget(QLabel("Surface normal:"), 3, 0)
        layout.addWidget(QLabel("nx"), 3, 1)
        layout.addWidget(self.reference_normal_x_value_label, 3, 2)
        layout.addWidget(QLabel("ny"), 3, 3)
        layout.addWidget(self.reference_normal_y_value_label, 3, 4)
        layout.addWidget(QLabel("nz"), 3, 5)
        layout.addWidget(self.reference_normal_z_value_label, 3, 6)

        layout.addWidget(QLabel("Plane samples:"), 4, 0)
        layout.addWidget(
            self.reference_normal_samples_value_label,
            4,
            1,
        )
        layout.addWidget(QLabel("RMSE [m]:"), 4, 2)
        layout.addWidget(self.reference_normal_rmse_value_label, 4, 3)
        layout.setColumnStretch(7, 1)
        return widget

    def _make_refine_tab(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        status_row = QHBoxLayout()
        status_row.addWidget(QLabel("Setup status:"))
        status_row.addWidget(self.reference_setup_status_label)
        status_row.addStretch()
        layout.addLayout(status_row)

        description = QLabel(
            "The refinement wizard enforces Safe Approach, Aligned "
            "Pre-approach, and live Surface Distance in order. Entering a "
            "stage never moves the robot."
        )
        description.setWordWrap(True)
        layout.addWidget(description)

        summary = QGroupBox("Workflow Summary")
        summary_layout = QFormLayout(summary)
        summary_layout.addRow(
            "Safe Approach Pose:",
            self.save_approach_status_label,
        )
        summary_layout.addRow(
            "Aligned Pre-approach Pose:",
            self.save_alignment_status_label,
        )
        summary_layout.addRow(
            "Probe Pose:",
            self.save_probe_status_label,
        )
        layout.addWidget(summary)
        layout.addWidget(self.start_probe_refinement_button)
        layout.addWidget(self.refinement_summary_status_label)
        layout.addStretch()
        return widget

    def _make_refine_stage_group(
        self,
        title,
        stage,
        status_label,
        move_button,
        approve_button,
        description,
    ):
        group = QGroupBox(title)
        layout = QHBoxLayout(group)

        details = QVBoxLayout()
        description_label = QLabel(description)
        description_label.setWordWrap(True)
        details.addWidget(description_label)

        status_row = QHBoxLayout()
        status_row.addWidget(QLabel("Status:"))
        status_row.addWidget(status_label)
        status_row.addStretch()
        details.addLayout(status_row)

        buttons = QHBoxLayout()
        buttons.addWidget(move_button)
        buttons.addWidget(approve_button)
        buttons.addStretch()
        details.addLayout(buttons)
        details.addStretch()
        layout.addLayout(details, 2)
        layout.addWidget(self._make_refinement_controls(stage), 1)
        return group

    def _make_refinement_controls(self, stage):
        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)

        movement_group = QGroupBox("Finite adjustments")
        movement_layout = QGridLayout(movement_group)
        buttons = self.refinement_buttons[stage]
        movement_layout.addWidget(buttons["up"], 0, 1)
        movement_layout.addWidget(buttons["left"], 1, 0)
        movement_layout.addWidget(buttons["right"], 1, 2)
        movement_layout.addWidget(buttons["down"], 2, 1)
        if "back" in buttons:
            movement_layout.addWidget(buttons["back"], 3, 0)
            movement_layout.addWidget(buttons["front"], 3, 2)
        movement_layout.addWidget(buttons["pitch_up"], 4, 0)
        movement_layout.addWidget(buttons["pitch_down"], 4, 2)
        movement_layout.addWidget(buttons["yaw_left"], 5, 0)
        movement_layout.addWidget(buttons["yaw_right"], 5, 2)
        layout.addWidget(movement_group)

        if stage == "probe":
            layout.addWidget(self._make_surface_distance_controls())
        layout.addStretch()
        return container

    def _make_surface_distance_controls(self, desired_distance_field=None):
        group = QGroupBox("Live surface distance")
        layout = QGridLayout(group)
        desired_distance_field = (
            desired_distance_field or self.reference_target_distance_field
        )
        layout.addWidget(QLabel("Desired [m]:"), 0, 0)
        layout.addWidget(desired_distance_field, 0, 1)
        layout.addWidget(QLabel("Measured [m]:"), 1, 0)
        layout.addWidget(self.live_surface_distance_value_label, 1, 1)
        layout.addWidget(QLabel("Delta [m]:"), 2, 0)
        layout.addWidget(self.surface_distance_delta_value_label, 2, 1)
        layout.addWidget(QLabel("Tolerance [m]:"), 3, 0)
        layout.addWidget(self.surface_distance_tolerance_field, 3, 1)
        layout.addWidget(self.test_surface_distance_button, 4, 0, 1, 2)
        layout.addWidget(
            self.surface_distance_test_status_label,
            5,
            0,
            1,
            2,
        )
        return group

    def _make_save_tab(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        approval_group = QGroupBox("Approval summary")
        approval_layout = QFormLayout(approval_group)
        approval_layout.addRow(
            "Approach pose:",
            self.save_approach_status_label,
        )
        approval_layout.addRow(
            "Aligned pre-approach pose:",
            self.save_alignment_status_label,
        )
        approval_layout.addRow(
            "Probe pose:",
            self.save_probe_status_label,
        )
        layout.addWidget(approval_group)

        definition_group = QGroupBox("Probe point definition")
        definition_layout = QFormLayout(definition_group)
        definition_layout.addRow(
            "Probe point ID:",
            self.probe_point_id_field,
        )
        definition_layout.addRow(
            "Display name:",
            self.probe_point_display_name_field,
        )
        definition_layout.addRow(
            "Position tolerance [m]:",
            self.probe_position_tolerance_field,
        )
        definition_layout.addRow(
            "Orientation tolerance [rad]:",
            self.probe_orientation_tolerance_field,
        )
        definition_layout.addRow(
            "Measurement duration [s]:",
            self.probe_measurement_duration_field,
        )
        layout.addWidget(definition_group)

        layout.addWidget(self.save_probe_point_status_label)
        layout.addStretch()
        return widget

    def _set_probe_setup_buttons_enabled(self, enabled):
        self.start_probe_refinement_button.setEnabled(bool(enabled))
        for button in (
            self.move_calculated_approach_button,
            self.use_current_approach_button,
            self.move_aligned_pose_button,
            self.use_current_alignment_button,
            self.test_surface_distance_button,
            self.approve_and_retract_button,
            self.retract_without_saving_button,
        ):
            button.setEnabled(False)
        for stage_buttons in self.refinement_buttons.values():
            for button in stage_buttons.values():
                button.setEnabled(False)
        if self._refinement_workflow_active:
            self._refresh_refinement_dialog()

    def _update_probe_setup_status_widgets(self):
        setup = self._probe_setup
        if setup is None:
            self.approach_step_status_label.setText("Waiting for target")
            self.alignment_step_status_label.setText("Waiting for target")
            self.probe_step_status_label.setText("Waiting for alignment")
            self.save_approach_status_label.setText("Not approved")
            self.save_alignment_status_label.setText("Not approved")
            self.save_probe_status_label.setText("Not approved")
            self.surface_distance_test_status_label.setText(
                "Waiting for aligned pre-approach approval"
            )
            self._update_save_probe_point_state()
            return

        approach_status = (
            "Approved" if setup.safe_approach_approved else "Calculated"
        )
        alignment_status = (
            "Approved"
            if setup.surface_alignment_approved
            else "Calculated"
        )
        if setup.probe_pose_approved:
            probe_status = "Approved"
        elif setup.surface_alignment_approved:
            probe_status = "Ready for refinement"
        else:
            probe_status = "Waiting for alignment"

        self.approach_step_status_label.setText(approach_status)
        self.alignment_step_status_label.setText(alignment_status)
        self.probe_step_status_label.setText(probe_status)
        self.save_approach_status_label.setText(
            "Approved" if setup.safe_approach_approved else "Not approved"
        )
        self.save_alignment_status_label.setText(
            "Approved"
            if setup.surface_alignment_approved
            else "Not approved"
        )
        self.save_probe_status_label.setText(
            "Approved" if setup.probe_pose_approved else "Not approved"
        )
        if not setup.surface_alignment_approved:
            self.surface_distance_test_status_label.setText(
                "Waiting for aligned pre-approach approval"
            )
        self._update_save_probe_point_state()

    def _update_save_probe_point_state(self, _value=None):
        setup = self._probe_setup
        approvals_complete = (
            setup is not None
            and setup.safe_approach_approved
            and setup.surface_alignment_approved
            and setup.probe_pose_approved
        )
        object_id = (
            self.saved_object_dropdown.currentData()
            if hasattr(self, "saved_object_dropdown")
            else None
        )
        routine_id = (
            self.saved_routine_dropdown.currentData()
            if hasattr(self, "saved_routine_dropdown")
            else None
        )
        routine = None
        if (
            object_id
            and routine_id
            and self._selected_definition is not None
            and self._selected_definition.object_id == object_id
        ):
            routine = self._selected_definition.get_routine(routine_id)

        point_id = self.probe_point_id_field.text().strip()
        display_name = self.probe_point_display_name_field.text().strip()
        numeric_ready = all(
            self._is_positive_number(field.text())
            for field in (
                self.probe_position_tolerance_field,
                self.probe_orientation_tolerance_field,
                self.probe_measurement_duration_field,
            )
        )
        provenance_ready = (
            self._selected_surface_point is not None
            and self._reference_view is not None
            and self._reference_view.view_id is not None
        )
        duplicate = (
            routine is not None
            and bool(point_id)
            and routine.get_probe_point(point_id) is not None
            and point_id != self._editing_probe_point_id
        )
        session = self._refinement_session
        workflow_ready = (
            self._refinement_workflow_active
            and session is not None
            and setup is not None
            and session.stage_is_approved(
                RefinementStage.SAFE_APPROACH
            )
            and session.stage_is_approved(RefinementStage.ALIGNMENT)
            and session.surface_distance_verified
            and session.pending_motion is None
            and not self._distance_failure_requires_retraction
            and routine is not None
            and bool(point_id)
            and bool(display_name)
            and numeric_ready
            and provenance_ready
            and not duplicate
        )
        self.approve_and_retract_button.setEnabled(workflow_ready)

        if session is not None and session.saved:
            status = "Probe point saved. Complete the required retraction."
        elif self._refinement_workflow_active and not (
            session is not None
            and session.stage_is_approved(
                RefinementStage.SAFE_APPROACH
            )
            and session.stage_is_approved(RefinementStage.ALIGNMENT)
        ):
            status = "Approve the safe and aligned poses in the wizard."
        elif self._refinement_workflow_active and not (
            session is not None and session.surface_distance_verified
        ):
            status = "Verify the live surface distance in the wizard."
        elif not approvals_complete and not self._refinement_workflow_active:
            status = "Complete the refinement wizard before saving."
        elif routine is None:
            status = "Select a saved object and routine."
        elif not provenance_ready:
            status = "Select a point in a captured reference view."
        elif not point_id or not display_name:
            status = "Enter a probe point ID and display name."
        elif duplicate:
            status = f"Probe point '{point_id}' already exists."
        elif not numeric_ready:
            status = "Enter positive tolerances and measurement duration."
        else:
            status = "Ready to save the approved probe point."
        self.save_probe_point_status_label.setText(status)

    @staticmethod
    def _is_positive_number(text):
        try:
            value = float(text.strip())
        except ValueError:
            return False
        return math.isfinite(value) and value > 0.0

    @staticmethod
    def _distance_validator(parent):
        validator = QDoubleValidator(0.001, 10.0, 3, parent)
        validator.setLocale(QLocale.c())
        validator.setNotation(QDoubleValidator.StandardNotation)
        return validator

    @staticmethod
    def _bounded_number_validator(
        parent,
        minimum,
        maximum,
        decimals,
    ):
        validator = QDoubleValidator(
            minimum,
            maximum,
            decimals,
            parent,
        )
        validator.setLocale(QLocale.c())
        validator.setNotation(QDoubleValidator.StandardNotation)
        return validator

    @staticmethod
    def _fixed_readout_label(text, width):
        label = QLabel(text)
        label.setFixedWidth(width)
        label.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Preferred)
        label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        return label

    @staticmethod
    def _refinement_button_label(action):
        labels = {
            "up": "Up",
            "down": "Down",
            "left": "Left",
            "right": "Right",
            "front": "Front",
            "back": "Back",
            "pitch_up": "Pitch Up",
            "pitch_down": "Pitch Down",
            "yaw_left": "Yaw Left",
            "yaw_right": "Yaw Right",
        }
        return labels[action]

    @staticmethod
    def _format_readout_value(value, decimals):
        rounding_threshold = 0.5 * 10 ** (-decimals)
        if abs(value) < rounding_threshold:
            value = 0.0
        return f"{value:.{decimals}f}"

    def _handle_reference_slot_point_changed(self, slot_index, u, v):
        capture = self._reference_slot_captures[slot_index]
        for candidate_index, widget in enumerate(
            self.reference_view_widgets
        ):
            if candidate_index == slot_index:
                continue
            widget.blockSignals(True)
            widget.clear_selection()
            widget.blockSignals(False)
        self._active_reference_slot = slot_index
        if capture is not None:
            self._reference_rgb_size = (
                capture.rgb_image.width,
                capture.rgb_image.height,
            )
            self._reference_depth_image = capture.depth_image
            self._reference_rgb_camera_info = (
                capture.rgb_camera_info
            )
            self._reference_camera_info = capture.depth_camera_info
            self._reference_view = capture.reference_view
            camera_name = REFERENCE_CAMERA_BY_ID[
                capture.camera_id
            ].display_name
            self.reference_pixel_value_label.setToolTip(camera_name)
        else:
            self._reference_rgb_size = None
            self._reference_depth_image = None
            self._reference_rgb_camera_info = None
            self._reference_camera_info = None
            self._reference_view = None
            self.reference_pixel_value_label.setToolTip(
                f"Camera slot {slot_index + 1}"
            )
        self._handle_reference_image_point_changed(u, v)

    def _handle_reference_slot_point_cleared(self, slot_index):
        if self._active_reference_slot == slot_index:
            self._handle_reference_image_point_cleared()
            self._active_reference_slot = None

    def _clear_all_reference_selections(self):
        for widget in self.reference_view_widgets:
            widget.blockSignals(True)
            widget.clear_selection()
            widget.blockSignals(False)
        self._active_reference_slot = None
        self._handle_reference_image_point_cleared()

    def _handle_reference_camera_selection_changed(self, slot_index):
        if slot_index >= len(self.reference_camera_dropdowns):
            return
        camera_id = (
            self.reference_camera_dropdowns[slot_index].currentData() or ""
        )
        self._reference_slot_captures[slot_index] = None
        widget = self.reference_view_widgets[slot_index]
        widget.clear_preview(
            f"Capture {REFERENCE_CAMERA_BY_ID[camera_id].display_name}"
            if camera_id
            else "No camera selected"
        )
        if self._active_reference_slot == slot_index:
            self._active_reference_slot = None
            self._reference_rgb_size = None
            self._reference_depth_image = None
            self._reference_rgb_camera_info = None
            self._reference_camera_info = None
            self._reference_view = None
            self._handle_reference_image_point_cleared()

    def _set_reference_camera_defaults(self):
        for slot_index, dropdown in enumerate(
            self.reference_camera_dropdowns
        ):
            dropdown.blockSignals(True)
            default_id = "hand" if slot_index == 0 else ""
            dropdown.setCurrentIndex(dropdown.findData(default_id))
            dropdown.blockSignals(False)

    def _clear_reference_camera_selections(self):
        for dropdown in self.reference_camera_dropdowns:
            dropdown.blockSignals(True)
            dropdown.setCurrentIndex(dropdown.findData(""))
            dropdown.blockSignals(False)

    def _selected_reference_camera_ids(self):
        return [
            str(dropdown.currentData() or "")
            for dropdown in self.reference_camera_dropdowns
        ]

    def _clear_reference_previews(self, message):
        self._reference_slot_captures = [None, None, None]
        self._active_reference_slot = None
        for slot_index, widget in enumerate(self.reference_view_widgets):
            camera_id = (
                self.reference_camera_dropdowns[slot_index].currentData()
                if slot_index < len(self.reference_camera_dropdowns)
                else ""
            )
            widget.clear_preview(
                message if camera_id else "No camera selected"
            )
        self._reference_rgb_size = None
        self._reference_depth_image = None
        self._reference_rgb_camera_info = None
        self._reference_camera_info = None
        self._reference_view = None
        self._handle_reference_image_point_cleared()

    def _handle_reference_image_point_changed(self, u, v):
        self.reference_pixel_value_label.setText(f"u={u}, v={v}")
        self.clear_reference_pixel_button.setEnabled(True)
        self._project_selected_reference_pixel(u, v)

    def _handle_reference_image_point_cleared(self):
        self.reference_pixel_value_label.setText("—")
        self.clear_reference_pixel_button.setEnabled(False)
        self._clear_selected_surface_point()

    def _handle_approach_mode_changed(self, _index=None):
        if self._selected_surface_point is None:
            self._clear_selected_approach_direction()
            return
        self._resolve_selected_approach_direction(
            self._selected_surface_point
        )

    def _handle_target_distance_changed(self):
        session = self._refinement_session
        if (
            self._refinement_workflow_active
            and session is not None
            and session.recovery_required
        ):
            self.reference_target_distance_field.setText(
                f"{session.target_surface_distance_m:.3f}"
            )
            self.reference_preapproach_distance_field.setText(
                f"{session.aligned_preapproach_distance_m:.3f}"
            )
            if hasattr(self, "refinement_dialog"):
                self.refinement_dialog.target_distance_field.setText(
                    self.reference_target_distance_field.text()
                )
                self.refinement_dialog.aligned_distance_field.setText(
                    self.reference_preapproach_distance_field.text()
                )
            self.refinement_recovery_status_label.setText(
                "Retract before changing either surface distance."
            )
            return
        if self._selected_approach_direction is None:
            self._clear_selected_surface_target()
            return
        self._resolve_selected_surface_target()

    def _handle_dialog_distances_changed(self):
        session = self._refinement_session
        old_target = (
            session.target_surface_distance_m
            if session is not None
            else self._distance_value(
                self.reference_target_distance_field,
                "Target surface distance",
            )
        )
        old_aligned = (
            session.aligned_preapproach_distance_m
            if session is not None
            else self._distance_value(
                self.reference_preapproach_distance_field,
                "Aligned pre-approach distance",
            )
        )
        try:
            if session is not None and session.recovery_required:
                raise ValueError("Retract before changing surface distances")
            target = self._distance_value(
                self.refinement_dialog.target_distance_field,
                "Desired surface distance",
            )
            aligned = self._distance_value(
                self.refinement_dialog.aligned_distance_field,
                "Aligned pre-approach distance",
            )
            if (
                aligned
                < target
                + MINIMUM_ALIGNED_PREAPPROACH_SEPARATION_M
                - 1e-12
            ):
                raise ValueError(
                    "Aligned pre-approach distance must be at least "
                    f"{MINIMUM_ALIGNED_PREAPPROACH_SEPARATION_M:.2f} m "
                    "greater than desired surface distance"
                )
        except Exception as exception:
            self.refinement_dialog.target_distance_field.setText(
                f"{old_target:.3f}"
            )
            self.refinement_dialog.aligned_distance_field.setText(
                f"{old_aligned:.3f}"
            )
            self.refinement_recovery_status_label.setText(str(exception))
            return False
        self.reference_target_distance_field.setText(f"{target:.3f}")
        self.reference_preapproach_distance_field.setText(
            f"{aligned:.3f}"
        )
        self._handle_target_distance_changed()
        self._refresh_refinement_dialog()
        return True

    def _handle_surface_tolerance_changed(self, _value=None):
        session = self._refinement_session
        if session is None:
            return
        session.surface_distance_verified = False
        self._update_save_probe_point_state()
        self._refresh_refinement_dialog()

    @property
    def selected_surface_point(self):
        """Return the transient projected reference surface point."""
        return self._selected_surface_point

    @property
    def selected_surface_normal(self):
        """Return the transient local surface-normal estimate."""
        return self._selected_surface_normal

    @property
    def selected_approach_direction(self):
        """Return the transient outward surface direction."""
        return self._selected_approach_direction

    @property
    def selected_surface_target(self):
        """Return the transient target and aligned pre-approach poses."""
        return self._selected_surface_target

    @property
    def selected_probe_setup(self):
        """Return the transient user-approved probe setup state."""
        return self._probe_setup

    def _clear_selected_surface_point(self):
        self._selected_surface_point = None
        self._clear_selected_surface_normal()
        self.reference_surface_frame_value_label.setText("—")
        self.reference_surface_frame_value_label.setToolTip("")
        self.reference_surface_x_value_label.setText("—")
        self.reference_surface_y_value_label.setText("—")
        self.reference_surface_z_value_label.setText("—")
        self.reference_depth_pixel_value_label.setText("—")
        self.reference_depth_pixel_value_label.setToolTip("")
        self._set_projection_status("No point")

    def _clear_selected_surface_normal(self):
        self._selected_surface_normal = None
        self._surface_normal_error = ""
        self._clear_selected_approach_direction()
        self.reference_normal_x_value_label.setText("—")
        self.reference_normal_y_value_label.setText("—")
        self.reference_normal_z_value_label.setText("—")
        self.reference_normal_samples_value_label.setText("—")
        self.reference_normal_rmse_value_label.setText("—")
        self._set_normal_status("No point")

    def _clear_selected_approach_direction(self):
        self._selected_approach_direction = None
        self._clear_selected_surface_target()
        self.reference_approach_source_value_label.setText("—")
        self.reference_approach_source_value_label.setToolTip("")
        self._set_approach_status("No point")

    def _clear_selected_surface_target(self):
        self._selected_surface_target = None
        self._calculated_probe_setup = None
        self._probe_setup = None
        if not self._refinement_workflow_active:
            self._refinement_session = None
        self.reference_target_x_value_label.setText("—")
        self.reference_target_y_value_label.setText("—")
        self.reference_target_z_value_label.setText("—")
        self.reference_target_roll_value_label.setText("—")
        self.reference_target_pitch_value_label.setText("—")
        self.reference_target_yaw_value_label.setText("—")
        self.reference_preapproach_x_value_label.setText("—")
        self.reference_preapproach_y_value_label.setText("—")
        self.reference_preapproach_z_value_label.setText("—")
        self.live_surface_distance_value_label.setText("—")
        self.surface_distance_delta_value_label.setText("—")
        self.surface_distance_test_status_label.setText(
            "Waiting for aligned pre-approach approval"
        )
        self._set_probe_setup_buttons_enabled(False)
        self._update_probe_setup_status_widgets()
        self._set_setup_status("No point")
        self._set_target_status("No point")

    def _set_setup_status(self, status, detail=""):
        self.reference_setup_status_label.setText(status)
        self.reference_setup_status_label.setToolTip(detail)

    def _set_target_status(self, status, detail=""):
        self.reference_target_status_label.setText(status)
        self.reference_target_status_label.setToolTip(detail)

    def _set_approach_status(self, status, detail=""):
        self.reference_approach_status_label.setText(status)
        self.reference_approach_status_label.setToolTip(detail)

    def _set_normal_status(self, status, detail=""):
        self.reference_normal_status_label.setText(status)
        self.reference_normal_status_label.setToolTip(detail)

    def _set_projection_status(self, status, detail=""):
        self.reference_projection_status_label.setText(status)
        self.reference_projection_status_label.setToolTip(detail)

    def _set_projection_unavailable(self, status, detail):
        self._selected_surface_point = None
        self._clear_selected_surface_normal()
        self.reference_surface_frame_value_label.setText("—")
        self.reference_surface_frame_value_label.setToolTip("")
        self.reference_surface_x_value_label.setText("—")
        self.reference_surface_y_value_label.setText("—")
        self.reference_surface_z_value_label.setText("—")
        self.reference_depth_pixel_value_label.setText("—")
        self.reference_depth_pixel_value_label.setToolTip("")
        self._set_projection_status(status, detail)

    def _project_selected_reference_pixel(self, u, v):
        if (
            self._reference_rgb_size is None
            or self._reference_depth_image is None
            or self._reference_rgb_camera_info is None
            or self._reference_camera_info is None
        ):
            self._set_projection_unavailable(
                "Depth unavailable",
                "The selected routine has no reference depth dataset.",
            )
            return
        try:
            result = project_reference_pixel(
                ImagePoint(u=u, v=v),
                self._reference_depth_image,
                self._reference_camera_info,
                rgb_size=self._reference_rgb_size,
                rgb_camera_info=self._reference_rgb_camera_info,
            )
        except ValueError as exception:
            self._set_projection_unavailable(
                "Unavailable",
                str(exception),
            )
            return

        self._selected_surface_point = result
        point = result.point_camera
        self.reference_surface_frame_value_label.setText(result.frame_id)
        self.reference_surface_frame_value_label.setToolTip(result.frame_id)
        self.reference_surface_x_value_label.setText(
            self._format_readout_value(point.x, 3)
        )
        self.reference_surface_y_value_label.setText(
            self._format_readout_value(point.y, 3)
        )
        self.reference_surface_z_value_label.setText(
            self._format_readout_value(point.z, 3)
        )
        mapped = result.mapped_pixel
        sampled = result.sampled_pixel
        if sampled == mapped:
            depth_source = f"u={mapped.u}, v={mapped.v}"
        else:
            depth_source = (
                f"{mapped.u},{mapped.v} → "
                f"{sampled.u},{sampled.v}"
            )
        mapping_detail = (
            f"RGB pixel u={result.requested_pixel.u}, "
            f"v={result.requested_pixel.v} in "
            f"{self._reference_rgb_size[0]}x"
            f"{self._reference_rgb_size[1]} mapped to depth pixel "
            f"u={mapped.u}, v={mapped.v} in "
            f"{self._reference_depth_image.width}x"
            f"{self._reference_depth_image.height}. "
            f"Sampled depth pixel u={sampled.u}, v={sampled.v}."
        )
        self.reference_depth_pixel_value_label.setText(depth_source)
        self.reference_depth_pixel_value_label.setToolTip(mapping_detail)
        self._set_projection_status("Ready", mapping_detail)
        self._estimate_selected_surface_normal(result)

    def _estimate_selected_surface_normal(self, projected_point):
        self._clear_selected_surface_normal()
        try:
            result = estimate_reference_surface_normal(
                projected_point,
                self._reference_depth_image,
                self._reference_camera_info,
            )
        except ValueError as exception:
            self._surface_normal_error = str(exception)
            self._set_normal_status(
                "Unavailable",
                self._surface_normal_error,
            )
            self._resolve_selected_approach_direction(projected_point)
            return

        self._selected_surface_normal = result
        self._surface_normal_error = ""
        normal = result.normal_camera
        self.reference_normal_x_value_label.setText(
            self._format_readout_value(normal.x, 3)
        )
        self.reference_normal_y_value_label.setText(
            self._format_readout_value(normal.y, 3)
        )
        self.reference_normal_z_value_label.setText(
            self._format_readout_value(normal.z, 3)
        )
        self.reference_normal_samples_value_label.setText(
            str(result.sample_count)
        )
        self.reference_normal_rmse_value_label.setText(
            self._format_readout_value(result.plane_rmse_m, 4)
        )
        self._set_normal_status(
            "Ready",
            f"Plane fit used a {result.neighborhood_radius_px} px "
            f"depth-image radius.",
        )
        self._resolve_selected_approach_direction(projected_point)

    def _resolve_selected_approach_direction(self, projected_point):
        self._clear_selected_approach_direction()
        if self._reference_view is None:
            self._set_approach_status(
                "Unavailable",
                "The selected routine has no saved reference-view pose.",
            )
            return
        if self._reference_view.controlled_frame != projected_point.frame_id:
            self._set_approach_status(
                "Frame mismatch",
                "The saved reference-view frame does not match the "
                "registered depth frame.",
            )
            return

        mode = self.reference_approach_mode_dropdown.currentData()
        try:
            result = resolve_reference_approach_direction(
                projected_point=projected_point,
                surface_normal=self._selected_surface_normal,
                controlled_frame_pose_object=(
                    self._reference_view.controlled_frame_pose_object
                ),
                mode=mode,
                surface_normal_unavailable_reason=(
                    self._surface_normal_error
                ),
            )
        except ValueError as exception:
            self._set_approach_status("Unavailable", str(exception))
            return

        self._selected_approach_direction = result
        source_text = {
            APPROACH_SOURCE_SURFACE_FIT: "Surface fit",
            APPROACH_SOURCE_TAG_X_SELECTED: "Calibrated tag +X",
        }[result.source]
        self.reference_approach_source_value_label.setText(source_text)
        detail = (
            "Tag +X is valid only when the tag mount is calibrated "
            "with +X pointing out of the inspected surface."
            if result.source == APPROACH_SOURCE_TAG_X_SELECTED
            else ""
        )
        self.reference_approach_source_value_label.setToolTip(detail)
        self._set_approach_status("Ready", detail)
        self._resolve_selected_surface_target()

    def _resolve_selected_surface_target(self):
        previous_setup = self._probe_setup
        if (
            self._selected_approach_direction is None
            or self._reference_view is None
        ):
            self._clear_selected_surface_target()
            return
        try:
            target_distance = self._distance_value(
                self.reference_target_distance_field,
                "Target surface distance",
            )
            aligned_preapproach_distance = self._distance_value(
                self.reference_preapproach_distance_field,
                "Aligned pre-approach distance",
            )
            result = resolve_reference_surface_target(
                approach_direction=self._selected_approach_direction,
                controlled_frame_pose_object=(
                    self._reference_view.controlled_frame_pose_object
                ),
                target_surface_distance_m=target_distance,
                aligned_preapproach_distance_m=(
                    aligned_preapproach_distance
                ),
            )
        except ValueError as exception:
            self._clear_selected_surface_target()
            self._set_target_status("Unavailable", str(exception))
            return

        self._selected_surface_target = result
        setup = initialize_reference_probe_setup(result)
        self._calculated_probe_setup = deepcopy(setup)
        if previous_setup is not None:
            if previous_setup.safe_approach_approved:
                setup = approve_safe_approach_pose(
                    setup,
                    previous_setup.safe_approach_pose_object,
                )
            self.surface_distance_test_status_label.setText(
                "Geometry changed. Reach and verify the aligned pose again."
            )
        self._probe_setup = setup
        if self._refinement_workflow_active:
            self._refinement_session = ProbeRefinementSession.create(
                self._calculated_probe_setup,
                self._probe_setup,
            )
            self._distance_failure_requires_retraction = False
            self._retraction_failed = False
        self._set_probe_setup_buttons_enabled(True)
        self._display_probe_setup("Calculated")
        if self._refinement_workflow_active:
            self.refinement_dialog.show_stage(
                self._refinement_session.active_stage
            )

    def _display_probe_setup(self, status):
        if self._probe_setup is None:
            self._clear_selected_surface_target()
            return
        target = self._probe_setup.probe_pose_object
        aligned = self._probe_setup.aligned_preapproach_pose_object
        self.reference_target_x_value_label.setText(
            self._format_readout_value(target.position.x, 3)
        )
        self.reference_target_y_value_label.setText(
            self._format_readout_value(target.position.y, 3)
        )
        self.reference_target_z_value_label.setText(
            self._format_readout_value(target.position.z, 3)
        )
        roll, pitch, yaw = quaternion_to_rpy(target.orientation)
        self.reference_target_roll_value_label.setText(
            self._format_readout_value(math.degrees(roll), 1)
        )
        self.reference_target_pitch_value_label.setText(
            self._format_readout_value(math.degrees(pitch), 1)
        )
        self.reference_target_yaw_value_label.setText(
            self._format_readout_value(math.degrees(yaw), 1)
        )
        self.reference_preapproach_x_value_label.setText(
            self._format_readout_value(aligned.position.x, 3)
        )
        self.reference_preapproach_y_value_label.setText(
            self._format_readout_value(aligned.position.y, 3)
        )
        self.reference_preapproach_z_value_label.setText(
            self._format_readout_value(aligned.position.z, 3)
        )
        detail = (
            "Object-frame probe quaternion: "
            f"x={target.orientation.x:.5f}, "
            f"y={target.orientation.y:.5f}, "
            f"z={target.orientation.z:.5f}, "
            f"w={target.orientation.w:.5f}. "
            "The probe local +X axis points outward from the surface."
        )
        for label in (
            self.reference_target_roll_value_label,
            self.reference_target_pitch_value_label,
            self.reference_target_yaw_value_label,
        ):
            label.setToolTip(detail)
        self._set_target_status("Ready", detail)
        self._set_setup_status(status, self._probe_setup_detail())
        self._set_probe_setup_buttons_enabled(True)
        self._update_probe_setup_status_widgets()

    def _probe_setup_detail(self):
        setup = self._probe_setup
        if setup is None:
            return "No transient probe setup is available."
        return (
            f"Approach approved={setup.safe_approach_approved}; "
            "aligned pre-approach approved="
            f"{setup.surface_alignment_approved}; "
            f"probe approved={setup.probe_pose_approved}. "
            "Nothing is persisted until Save Probe Point is pressed."
        )

    def handle_start_probe_refinement(self):
        """Open a new supervised draft without commanding movement."""
        try:
            if self._calculated_probe_setup is None:
                raise ValueError(
                    "Select a valid reference point before refinement"
                )
            setup = self._require_probe_setup()
            self._refinement_session = ProbeRefinementSession.create(
                self._calculated_probe_setup,
                setup,
            )
            if not setup.safe_approach_approved:
                self._refinement_session.seed_safe_approach_from_current_pose(
                    self._current_probe_pose_object()
                )
        except Exception as exception:
            self._show_setup_error(
                "Start Probe Point Refinement",
                exception,
            )
            return False

        self._refinement_workflow_active = True
        self._distance_failure_requires_retraction = False
        self._retraction_failed = False
        self._hand_depth_history.clear()
        self.inspection_workspace_splitter.setEnabled(False)
        self.refinement_dialog.open_for_stage(
            self._refinement_session.active_stage
        )
        self._set_status_text("Probe refinement workflow started")
        return True

    def request_close_refinement_workflow(self):
        """Close only when no movement or recovery remains active."""
        session = self._refinement_session
        if session is None:
            self._finish_refinement_workflow_close()
            return True
        if session.pending_motion is not None:
            self.refinement_recovery_status_label.setText(
                "Wait for the active movement and settle check."
            )
            return False
        if session.recovery_required:
            self.refinement_recovery_status_label.setText(
                "Retract Without Saving is required before closing."
            )
            return False
        session.discard_unapproved_candidates()
        self._finish_refinement_workflow_close()
        return True

    def _finish_refinement_workflow_close(self):
        self._refinement_workflow_active = False
        self._probe_motion_pending = False
        self._distance_failure_requires_retraction = False
        self._retraction_failed = False
        if hasattr(self, "inspection_workspace_splitter"):
            self.inspection_workspace_splitter.setEnabled(True)
        self._refinement_session = None
        self.refinement_summary_status_label.setText(
            "Refinement workflow closed. Persisted data was preserved."
        )
        self._update_probe_setup_status_widgets()

    def handle_refinement_back(self):
        """Navigate backward without commanding movement."""
        session = self._require_refinement_session()
        if session.recovery_required:
            self.refinement_recovery_status_label.setText(
                "Retract Without Saving before navigating backward."
            )
            return False
        index = ProbeRefinementDialog.STAGES.index(session.active_stage)
        if index == 0:
            return False
        self.refinement_dialog.show_stage(
            ProbeRefinementDialog.STAGES[index - 1]
        )
        return True

    def handle_refinement_next(self):
        """Advance only when the current pose has been approved."""
        session = self._require_refinement_session()
        stage = session.active_stage
        if not session.stage_is_approved(stage):
            self.refinement_recovery_status_label.setText(
                "Approve the current stage before continuing."
            )
            return False
        index = ProbeRefinementDialog.STAGES.index(stage)
        if index >= len(ProbeRefinementDialog.STAGES) - 1:
            return False
        self.refinement_dialog.show_stage(
            ProbeRefinementDialog.STAGES[index + 1]
        )
        return True

    def _handle_refinement_stage_changed(self, stage):
        session = self._refinement_session
        if session is None:
            return
        session.active_stage = stage
        self.refinement_recovery_status_label.setText("")

    def _refresh_refinement_dialog(self):
        session = self._refinement_session
        if session is None or not hasattr(self, "refinement_dialog"):
            return
        for stage in RefinementStage:
            labels = self.refinement_dialog.pose_comparison_labels[stage]
            calculated = session.calculated_pose(stage)
            candidate = session.candidate_pose(stage)
            approved = session.approved_pose(stage)
            labels["calculated"].setText(self._pose_summary(calculated))
            labels["candidate"].setText(self._pose_summary(candidate))
            labels["approved"].setText(
                self._pose_summary(approved)
                if approved is not None
                else "Not set"
            )
            labels["difference"].setText(
                self._pose_difference_summary(calculated, candidate)
            )
            if approved is None:
                status = (
                    "Not set"
                    if self._poses_equivalent(calculated, candidate)
                    else "Modified"
                )
            else:
                status = (
                    "Approved"
                    if self._poses_equivalent(approved, candidate)
                    else "Modified"
                )
            labels["status"].setText(status)

        self.approach_step_status_label.setText(
            session.motion_states[RefinementStage.SAFE_APPROACH].value
        )
        self.alignment_step_status_label.setText(
            session.motion_states[RefinementStage.ALIGNMENT].value
        )
        probe_state = session.motion_states[RefinementStage.PROBE].value
        if session.surface_distance_verified:
            probe_state = "Surface Distance Verified"
        self.probe_step_status_label.setText(probe_state)

        pending = session.pending_motion is not None
        current = session.active_stage
        recovery_only = self._retraction_failed
        safe_page = current == RefinementStage.SAFE_APPROACH
        alignment_page = current == RefinementStage.ALIGNMENT
        probe_page = current == RefinementStage.PROBE
        safe_reached = (
            session.motion_states[RefinementStage.SAFE_APPROACH]
            == RefinementMotionState.REACHED
        )
        alignment_reached = (
            session.motion_states[RefinementStage.ALIGNMENT]
            == RefinementMotionState.REACHED
        )

        safe_enabled = safe_page and not pending and not recovery_only
        self.move_calculated_approach_button.setEnabled(safe_enabled)
        self.use_current_approach_button.setEnabled(safe_enabled)
        for button in self.refinement_buttons["approach"].values():
            button.setEnabled(safe_enabled and safe_reached)

        alignment_enabled = (
            alignment_page
            and safe_reached
            and not pending
            and not recovery_only
        )
        self.move_aligned_pose_button.setEnabled(alignment_enabled)
        self.use_current_alignment_button.setEnabled(alignment_enabled)
        for button in self.refinement_buttons["alignment"].values():
            button.setEnabled(alignment_enabled and alignment_reached)

        test_enabled = (
            probe_page
            and alignment_reached
            and not pending
            and not self._distance_failure_requires_retraction
            and not recovery_only
        )
        self.test_surface_distance_button.setEnabled(test_enabled)
        self.retract_without_saving_button.setEnabled(
            session.recovery_required and not pending
        )
        distances_editable = not pending and not session.recovery_required
        self.reference_target_distance_field.setEnabled(distances_editable)
        self.reference_preapproach_distance_field.setEnabled(
            distances_editable
        )
        if not self.refinement_dialog.target_distance_field.hasFocus():
            self.refinement_dialog.target_distance_field.setText(
                self.reference_target_distance_field.text()
            )
        if not self.refinement_dialog.aligned_distance_field.hasFocus():
            self.refinement_dialog.aligned_distance_field.setText(
                self.reference_preapproach_distance_field.text()
            )
        self.refinement_dialog.target_distance_field.setEnabled(
            distances_editable
        )
        self.refinement_dialog.aligned_distance_field.setEnabled(
            distances_editable
        )
        self.surface_distance_tolerance_field.setEnabled(not pending)
        self.refine_translation_step_field.setEnabled(not pending)
        self.refine_rotation_step_field.setEnabled(not pending)
        self.refine_frame_dropdown.setEnabled(not pending)

        self.refinement_dialog.back_button.setEnabled(
            current != RefinementStage.SAFE_APPROACH
            and not pending
            and not session.recovery_required
        )
        self.refinement_dialog.next_button.setVisible(not probe_page)
        approved = session.approved_pose(current)
        self.refinement_dialog.next_button.setEnabled(
            session.stage_is_approved(current) and not pending
        )
        self.refinement_dialog.next_button.setText(
            "Keep Existing and Continue"
            if (
                approved is not None
                and self._poses_equivalent(
                    approved,
                    session.candidate_pose(current),
                )
            )
            else "Next"
        )
        self.refinement_dialog.close_button.setEnabled(not pending)
        if session.recovery_required:
            self.refinement_recovery_status_label.setText(
                session.recovery_message
            )
        self._update_save_probe_point_state()

    @staticmethod
    def _pose_summary(pose):
        roll, pitch, yaw = quaternion_to_rpy(pose.orientation)
        return (
            f"p=({pose.position.x:.4f}, {pose.position.y:.4f}, "
            f"{pose.position.z:.4f}) m; rpy=("
            f"{math.degrees(roll):.2f}, "
            f"{math.degrees(pitch):.2f}, "
            f"{math.degrees(yaw):.2f}) deg"
        )

    @staticmethod
    def _pose_difference_summary(reference, candidate):
        reference_rpy = quaternion_to_rpy(reference.orientation)
        candidate_rpy = quaternion_to_rpy(candidate.orientation)
        rotation_delta = [
            math.degrees(
                math.atan2(
                    math.sin(candidate_value - reference_value),
                    math.cos(candidate_value - reference_value),
                )
            )
            for reference_value, candidate_value in zip(
                reference_rpy,
                candidate_rpy,
            )
        ]
        return (
            f"dp=({candidate.position.x - reference.position.x:+.4f}, "
            f"{candidate.position.y - reference.position.y:+.4f}, "
            f"{candidate.position.z - reference.position.z:+.4f}) m; "
            f"drpy=({rotation_delta[0]:+.2f}, "
            f"{rotation_delta[1]:+.2f}, "
            f"{rotation_delta[2]:+.2f}) deg"
        )

    @staticmethod
    def _poses_equivalent(first, second):
        position_error = math.sqrt(
            (first.position.x - second.position.x) ** 2
            + (first.position.y - second.position.y) ** 2
            + (first.position.z - second.position.z) ** 2
        )
        dot = abs(
            first.orientation.x * second.orientation.x
            + first.orientation.y * second.orientation.y
            + first.orientation.z * second.orientation.z
            + first.orientation.w * second.orientation.w
        )
        dot = max(-1.0, min(1.0, dot))
        angle_error = 2.0 * math.acos(dot)
        return position_error <= 1e-9 and angle_error <= 1e-9

    def _require_refinement_session(self):
        session = self._refinement_session
        if session is None or not self._refinement_workflow_active:
            raise RuntimeError("Probe refinement workflow is not active")
        return session

    def handle_approve_and_retract(self):
        """Capture, atomically save, then command mandatory retraction."""
        try:
            self._require_command_path_idle(require_settled=True)
            session = self._require_refinement_session()
            if not session.surface_distance_verified:
                raise ValueError("Surface Distance Verified is required")
            if not session.stage_is_approved(
                RefinementStage.SAFE_APPROACH
            ):
                raise ValueError("Safe Approach Pose is not approved")
            if not session.stage_is_approved(
                RefinementStage.ALIGNMENT
            ):
                raise ValueError(
                    "Aligned Pre-approach Pose is not approved"
                )
            current = self._current_probe_pose_object()
            session.approve(RefinementStage.PROBE, current)
            self._probe_setup = approve_probe_pose(
                self._require_probe_setup(),
                current,
            )
            result = self._persist_probe_point(self._probe_setup)
            session.saved = True
            session.require_recovery(
                "Probe point saved. Retraction to the aligned "
                "pre-approach pose is required."
            )
            target = session.candidate_pose(RefinementStage.ALIGNMENT)
            if not self._send_refinement_motion(
                RefinementStage.ALIGNMENT,
                "retraction",
                target,
                updates_candidate=False,
            ):
                self._retraction_failed = True
                raise RuntimeError(
                    "Probe point was saved, but retraction could not start"
                )
        except Exception as exception:
            self.save_probe_point_status_label.setText(str(exception))
            self.refinement_recovery_status_label.setText(str(exception))
            self.show_warning("Approve and Retract", str(exception))
            self._refresh_refinement_dialog()
            return False

        self.save_probe_point_status_label.setText(
            f"Saved probe point '{result[2]}'; retracting."
        )
        self._refresh_refinement_dialog()
        return True

    def handle_retract_without_saving(self):
        """Return to the current derived aligned pose without persistence."""
        try:
            session = self._require_refinement_session()
            if not session.recovery_required:
                raise ValueError("No retraction is currently required")
            target = session.candidate_pose(RefinementStage.ALIGNMENT)
            if not self._send_refinement_motion(
                RefinementStage.ALIGNMENT,
                "retraction",
                target,
                updates_candidate=False,
            ):
                return False
        except Exception as exception:
            self._show_setup_error("Retract Without Saving", exception)
            return False
        return True

    def _persist_probe_point(self, setup):
        object_id = self.saved_object_dropdown.currentData()
        routine_id = self.saved_routine_dropdown.currentData()
        if not object_id or not routine_id:
            raise ValueError(
                "Select a saved object and routine before saving"
            )
        probe_point = self._build_probe_point(setup)
        if self._editing_probe_point_id is None:
            stored_definition = self.object_repository.add_probe_point(
                object_id,
                routine_id,
                probe_point,
            )
        else:
            if probe_point.probe_point_id != self._editing_probe_point_id:
                raise ValueError(
                    "Probe point ID cannot change while replacing a point"
                )
            stored_definition = self.object_repository.replace_probe_point(
                object_id,
                routine_id,
                probe_point,
            )
        self._selected_definition = stored_definition
        self._set_status_text(
            f"Saved probe point '{object_id}/{routine_id}/"
            f"{probe_point.probe_point_id}'"
        )
        return object_id, routine_id, probe_point.probe_point_id

    def _build_probe_point(self, setup):
        if (
            self._selected_surface_point is None
            or self._reference_view is None
            or self._reference_view.view_id is None
        ):
            raise ValueError(
                "Select a point in a captured reference view"
            )
        probe_point_id = self.probe_point_id_field.text().strip()
        display_name = self.probe_point_display_name_field.text().strip()
        if not probe_point_id:
            raise ValueError("Probe point ID must not be empty")
        if not display_name:
            raise ValueError("Probe point display name must not be empty")
        surface_target = setup.surface_target
        probe_point = ProbePoint(
            probe_point_id=probe_point_id,
            display_name=display_name,
            safe_approach_pose_object=deepcopy(
                setup.safe_approach_pose_object
            ),
            probe_pose_object=deepcopy(setup.probe_pose_object),
            target_surface_distance_m=(
                surface_target.target_surface_distance_m
            ),
            position_tolerance_m=self._distance_value(
                self.probe_position_tolerance_field,
                "Position tolerance",
            ),
            orientation_tolerance_rad=self._distance_value(
                self.probe_orientation_tolerance_field,
                "Orientation tolerance",
            ),
            measurement_duration_sec=self._distance_value(
                self.probe_measurement_duration_field,
                "Measurement duration",
            ),
            aligned_preapproach_distance_m=(
                surface_target.aligned_preapproach_distance_m
            ),
            reference_pixel=deepcopy(
                self._selected_surface_point.requested_pixel
            ),
            reference_view_id=self._reference_view.view_id,
        )
        probe_point.validate()
        return probe_point

    def handle_move_to_approach_pose(self):
        session = self._require_refinement_session()
        return self._send_refinement_motion(
            RefinementStage.SAFE_APPROACH,
            "safe approach candidate",
            session.candidate_pose(RefinementStage.SAFE_APPROACH),
        )

    def handle_move_to_aligned_pose(self):
        session = self._require_refinement_session()
        return self._send_refinement_motion(
            RefinementStage.ALIGNMENT,
            "aligned pre-approach candidate",
            session.candidate_pose(RefinementStage.ALIGNMENT),
        )

    def handle_move_to_probe_pose(self):
        self._show_setup_error(
            "Move to Probe Pose",
            ValueError(
                "Direct probe-pose movement is disabled. Use Test Surface "
                "Distance from the reached aligned pre-approach pose."
            ),
        )
        return False

    def handle_refine_pose(self, stage, action):
        try:
            session = self._require_refinement_session()
            stage_value = {
                "approach": RefinementStage.SAFE_APPROACH,
                "alignment": RefinementStage.ALIGNMENT,
            }.get(stage)
            if stage_value is None:
                raise ValueError(
                    "Probe geometry is refined only at the aligned "
                    "pre-approach pose"
                )
            if session.active_stage != stage_value:
                raise ValueError("The requested refinement stage is inactive")
            if (
                session.motion_states[stage_value]
                != RefinementMotionState.REACHED
            ):
                raise ValueError("Reach the current candidate before adjusting")
            translation_step = self._bounded_positive_value(
                self.refine_translation_step_field,
                "Translation step",
                MAX_REFINEMENT_TRANSLATION_M,
            )
            rotation_step = math.radians(
                self._bounded_positive_value(
                    self.refine_rotation_step_field,
                    "Rotation step",
                    MAX_REFINEMENT_ROTATION_DEG,
                )
            )
            translation, pitch, yaw = self._refinement_delta(
                action,
                translation_step,
                rotation_step,
            )
            if not self._send_refinement_relative_motion(
                stage_value,
                f"{stage} {action.replace('_', ' ')} refinement",
                translation,
                pitch,
                yaw,
            ):
                return False
        except Exception as exception:
            self._show_setup_error("Refine Probe Pose", exception)
            return False

        self._set_setup_status(
            "Refinement sent",
            "The achieved sensor-tip pose will update the draft after "
            "movement and settling succeed.",
        )
        self._refresh_refinement_dialog()
        return True

    def handle_test_surface_distance(self):
        measurement_started = False
        try:
            self._require_command_path_idle(require_settled=True)
            session = self._require_refinement_session()
            if session.active_stage != RefinementStage.PROBE:
                raise ValueError("Open the Probe Pose stage first")
            if self._distance_failure_requires_retraction:
                raise ValueError(
                    "Retract after the failed distance measurement before "
                    "retrying"
                )
            if (
                session.motion_states[RefinementStage.ALIGNMENT]
                != RefinementMotionState.REACHED
            ):
                raise ValueError(
                    "Reach the current aligned pre-approach pose first"
                )
            target_distance = self._distance_value(
                self.reference_target_distance_field,
                "Desired surface distance",
            )
            tolerance = self._bounded_positive_value(
                self.surface_distance_tolerance_field,
                "Surface-distance tolerance",
                0.05,
            )
            maximum_step = min(
                self._bounded_positive_value(
                    self.refine_translation_step_field,
                    "Translation step",
                    MAX_REFINEMENT_TRANSLATION_M,
                ),
                MAX_SURFACE_CORRECTION_STEP_M,
            )
            measurement_started = True
            samples = self._measure_live_surface_distance_samples()
            aggregate = aggregate_surface_distance_samples(
                samples,
                target_distance,
                maximum_step,
                tolerance_m=tolerance,
                minimum_samples=3,
                minimum_span_sec=SURFACE_DISTANCE_SAMPLE_WINDOW_SEC,
                stability_tolerance_m=(
                    SURFACE_DISTANCE_STABILITY_TOLERANCE_M
                ),
            )
            correction = aggregate.correction
            self.live_surface_distance_value_label.setText(
                self._format_readout_value(aggregate.distance_m, 4)
            )
            self.surface_distance_delta_value_label.setText(
                self._format_readout_value(correction.error_m, 4)
            )
            quality = (
                f"{aggregate.sample_count} frames over "
                f"{aggregate.sample_span_sec:.3f} s, peak-to-peak "
                f"{aggregate.peak_to_peak_m:.4f} m"
            )
            if aggregate.verified:
                current = self._current_probe_pose_object()
                session.mark_surface_verified(current)
                self.surface_distance_test_status_label.setText(
                    f"Surface Distance Verified within {tolerance:.4f} m. "
                    f"{quality}. Explicit approval is still required."
                )
                self._refresh_refinement_dialog()
                return True

            current = self._current_probe_pose_object()
            target = refine_probe_pose(
                current,
                current.orientation,
                local_translation=Vector3Data(
                    x=-correction.inward_correction_m,
                    y=0.0,
                    z=0.0,
                ),
            )
            direction = (
                "inward"
                if correction.inward_correction_m > 0.0
                else "outward"
            )
            if not self._send_refinement_motion(
                RefinementStage.PROBE,
                f"surface-distance {direction} correction",
                target,
                axial_correction_m=correction.inward_correction_m,
            ):
                if session.recovery_required:
                    self._distance_failure_requires_retraction = True
                    self._refresh_refinement_dialog()
                return False
        except Exception as exception:
            session = self._refinement_session
            if (
                measurement_started
                and session is not None
                and session.recovery_required
            ):
                session.require_recovery(
                    f"Distance Measurement Failed: {exception}"
                )
                self._distance_failure_requires_retraction = True
            self.surface_distance_test_status_label.setText(
                f"Unavailable: {exception}"
            )
            self._show_setup_error("Test Surface Distance", exception)
            self._refresh_refinement_dialog()
            return False

        self.surface_distance_test_status_label.setText(
            f"Measured {aggregate.distance_m:.4f} m, target "
            f"{target_distance:.4f} m. Sent one "
            f"{abs(correction.inward_correction_m):.4f} m {direction} "
            f"correction. {quality}. Measure again after settling."
        )
        self._refresh_refinement_dialog()
        return True

    @staticmethod
    def _refinement_delta(action, translation_step, rotation_step):
        translations = {
            "up": Vector3Data(x=0.0, y=0.0, z=translation_step),
            "down": Vector3Data(x=0.0, y=0.0, z=-translation_step),
            "left": Vector3Data(x=0.0, y=translation_step, z=0.0),
            "right": Vector3Data(x=0.0, y=-translation_step, z=0.0),
            "front": Vector3Data(x=-translation_step, y=0.0, z=0.0),
            "back": Vector3Data(x=translation_step, y=0.0, z=0.0),
        }
        if action in translations:
            return translations[action], 0.0, 0.0
        rotations = {
            "pitch_up": (-rotation_step, 0.0),
            "pitch_down": (rotation_step, 0.0),
            "yaw_left": (0.0, rotation_step),
            "yaw_right": (0.0, -rotation_step),
        }
        if action not in rotations:
            raise ValueError(f"Unknown refinement action: {action}")
        pitch, yaw = rotations[action]
        return Vector3Data.zero(), pitch, yaw

    def _move_setup_pose(self, attribute, label):
        try:
            setup = self._require_probe_setup()
            pose = getattr(setup, attribute)
        except Exception as exception:
            self._show_setup_error("Move Probe Setup", exception)
            return
        self._move_transient_probe_pose(pose, label)

    def handle_use_current_as_approach(self):
        try:
            self._require_command_path_idle(require_settled=True)
            session = self._require_refinement_session()
            if session.active_stage != RefinementStage.SAFE_APPROACH:
                raise ValueError("Open the Safe Approach Pose stage first")
            current = self._current_probe_pose_object()
            session.approve(RefinementStage.SAFE_APPROACH, current)
            session.motion_states[RefinementStage.SAFE_APPROACH] = (
                RefinementMotionState.REACHED
            )
            self._probe_setup = approve_safe_approach_pose(
                self._require_probe_setup(),
                current,
            )
        except Exception as exception:
            self._show_setup_error("Capture Approach Pose", exception)
            return
        self._display_probe_setup("Approach approved")
        self._set_status_text("Current probe pose approved as approach pose")
        self._refresh_refinement_dialog()
        return True

    def handle_use_current_alignment(self):
        try:
            self._require_command_path_idle(require_settled=True)
            session = self._require_refinement_session()
            if session.active_stage != RefinementStage.ALIGNMENT:
                raise ValueError(
                    "Open the Aligned Pre-approach Pose stage first"
                )
            if (
                session.motion_states[RefinementStage.SAFE_APPROACH]
                != RefinementMotionState.REACHED
            ):
                raise ValueError(
                    "Reach the safe approach during this workflow first"
                )
            current = self._current_probe_pose_object()
            session.approve(RefinementStage.ALIGNMENT, current)
            session.motion_states[RefinementStage.ALIGNMENT] = (
                RefinementMotionState.REACHED
            )
            self._probe_setup = approve_surface_alignment_pose(
                self._require_probe_setup(),
                current,
            )
        except Exception as exception:
            self._show_setup_error(
                "Capture Aligned Pre-approach Pose",
                exception,
            )
            return
        self._display_probe_setup("Aligned pre-approach approved")
        self._set_status_text(
            "Current probe pose approved as aligned pre-approach"
        )
        self._refresh_refinement_dialog()
        return True

    def handle_use_current_as_probe(self):
        self._show_setup_error(
            "Capture Probe Pose",
            ValueError(
                "Probe approval requires verified live surface distance "
                "and Approve and Retract"
            ),
        )
        return False

    def _move_transient_probe_pose(self, probe_pose_object, label):
        session = self._require_refinement_session()
        return self._send_refinement_motion(
            session.active_stage,
            label,
            probe_pose_object,
        )

    def _send_refinement_motion(
        self,
        stage,
        label,
        probe_pose_object,
        updates_candidate=True,
        axial_correction_m=0.0,
    ):
        try:
            self._require_command_path_idle()
            session = self._require_refinement_session()
            if self._retraction_failed and label != "retraction":
                raise RuntimeError(
                    "Only retraction is permitted after retraction failure"
                )
            command = self._build_probe_pose_command(probe_pose_object)
            motion = PendingRefinementMotion(
                request_id=command.command.request_id,
                stage=stage,
                purpose=label,
                target_pose_object=deepcopy(probe_pose_object),
                updates_candidate=updates_candidate,
                axial_correction_m=axial_correction_m,
                command_id=CommandID.MOVE_ARM_TO_TAG,
                verify_achieved_pose=True,
            )
            session.begin_motion(motion)
        except Exception as exception:
            self._show_setup_error("Move Probe Setup", exception)
            return False
        try:
            self.complex_command_publisher.publish(command)
        except Exception as exception:
            session.fail_motion(motion.request_id, str(exception))
            self._show_setup_error("Move Probe Setup", exception)
            return False
        self._probe_motion_pending = True
        self._set_status_text(f"Command sent: move to {label}")
        self._refresh_refinement_dialog()
        return True

    def _send_refinement_relative_motion(
        self,
        stage,
        label,
        translation,
        pitch_rad,
        yaw_rad,
    ):
        try:
            self._require_command_path_idle()
            session = self._require_refinement_session()
            if self._retraction_failed:
                raise RuntimeError(
                    "Only retraction is permitted after retraction failure"
                )
            command = self._new_command(CommandID.MOVE_ARM_RELATIVE)
            command.offset.header.frame_id = (
                self._selected_refinement_frame_id()
            )
            command.offset.pose.position.x = translation.x
            command.offset.pose.position.y = translation.y
            command.offset.pose.position.z = translation.z
            self._write_quaternion_message(
                command.offset.pose.orientation,
                self._relative_rotation_quaternion(pitch_rad, yaw_rad),
            )
            motion = PendingRefinementMotion(
                request_id=command.command.request_id,
                stage=stage,
                purpose=label,
                target_pose_object=session.candidate_pose(stage),
                updates_candidate=False,
                command_id=CommandID.MOVE_ARM_RELATIVE,
                verify_achieved_pose=False,
            )
            session.begin_motion(motion)
        except Exception as exception:
            self._show_setup_error("Move Probe Setup", exception)
            return False
        try:
            self.complex_command_publisher.publish(command)
        except Exception as exception:
            session.fail_motion(motion.request_id, str(exception))
            self._show_setup_error("Move Probe Setup", exception)
            return False
        self._probe_motion_pending = True
        self._set_status_text(
            f"Command sent: {label} in "
            f"{command.offset.header.frame_id}"
        )
        self._refresh_refinement_dialog()
        return True

    def _selected_refinement_frame_id(self):
        selection = self.refine_frame_dropdown.currentData()
        if selection == REFINEMENT_FRAME_SENSOR:
            return self._active_probe_frame()
        if selection == REFINEMENT_FRAME_HAND:
            return HAND_FRAME_NAME
        if selection == REFINEMENT_FRAME_TAG:
            if self._selected_definition is None:
                raise ValueError("No inspection object is selected")
            tag_id = self._selected_definition.reference_tag.tag_id
            return f"{TagFrames.SPOT_FRAME_FILTERED.value}{tag_id}"
        if selection == REFINEMENT_FRAME_BODY:
            return "body"
        if selection == REFINEMENT_FRAME_MAP:
            return "map"
        raise ValueError("Select a valid refinement frame")

    @staticmethod
    def _relative_rotation_quaternion(pitch_rad, yaw_rad):
        half_pitch = pitch_rad * 0.5
        half_yaw = yaw_rad * 0.5
        return QuaternionData(
            x=-math.sin(half_pitch) * math.sin(half_yaw),
            y=math.sin(half_pitch) * math.cos(half_yaw),
            z=math.cos(half_pitch) * math.sin(half_yaw),
            w=math.cos(half_pitch) * math.cos(half_yaw),
        )

    def _complete_pending_refinement_motion(self, request_id):
        session = self._refinement_session
        if session is None or session.pending_motion is None:
            return
        motion = session.pending_motion
        if motion.request_id != request_id:
            return
        try:
            if not motion.verify_achieved_pose:
                session.complete_relative_motion(request_id)
                self._probe_motion_pending = False
                self._hand_depth_history.clear()
                self._set_status_text(
                    f"Reached {motion.purpose}; capture the current pose "
                    "when approving"
                )
                self._refresh_refinement_dialog()
                return
            achieved = self._current_probe_pose_object()
            position_tolerance = self._distance_value(
                self.probe_position_tolerance_field,
                "Position tolerance",
            )
            orientation_tolerance = self._distance_value(
                self.probe_orientation_tolerance_field,
                "Orientation tolerance",
            )
            position_error, orientation_error = self._pose_errors(
                motion.target_pose_object,
                achieved,
            )
            if position_error > position_tolerance:
                raise RuntimeError(
                    "Achieved probe position missed the target by "
                    f"{position_error:.4f} m"
                )
            if orientation_error > orientation_tolerance:
                raise RuntimeError(
                    "Achieved probe orientation missed the target by "
                    f"{math.degrees(orientation_error):.2f} deg"
                )
            session.complete_motion(request_id, achieved)
            if motion.purpose == "retraction":
                session.complete_retraction()
                self._distance_failure_requires_retraction = False
                self._retraction_failed = False
                self.surface_distance_test_status_label.setText(
                    "Retraction reached the aligned pre-approach pose."
                )
                if session.saved:
                    self.save_probe_point_status_label.setText(
                        "Probe point saved and retraction completed."
                    )
            else:
                self.surface_distance_test_status_label.setText(
                    "Movement reached and settled."
                    if motion.stage == RefinementStage.PROBE
                    else self.surface_distance_test_status_label.text()
                )
        except Exception as exception:
            self._fail_pending_refinement_motion(
                str(exception),
                request_id,
            )
            return

        self._probe_motion_pending = False
        self._hand_depth_history.clear()
        self._set_status_text(
            f"Reached {motion.purpose}; achieved pose verified"
        )
        self._refresh_refinement_dialog()

    def _fail_pending_refinement_motion(
        self,
        message,
        request_id=None,
    ):
        session = self._refinement_session
        motion = session.pending_motion if session is not None else None
        if session is not None:
            if motion is None:
                return
            result_request_id = request_id or motion.request_id
            if result_request_id != motion.request_id:
                return
            session.fail_motion(result_request_id, message)
            if motion is not None and motion.purpose == "retraction":
                session.require_recovery(
                    f"Retraction Failed: {message}"
                )
                self._retraction_failed = True
            elif (
                motion is not None
                and motion.stage == RefinementStage.PROBE
            ):
                session.require_recovery(
                    f"Probe-axis Movement Failed: {message}"
                )
                self._distance_failure_requires_retraction = True
        self._probe_motion_pending = False
        self.refinement_recovery_status_label.setText(message)
        self._set_status_text(f"Probe refinement movement failed: {message}")
        self._refresh_refinement_dialog()

    def _clear_refinement_execution_evidence(self, message):
        session = self._refinement_session
        if session is None:
            return
        session.surface_distance_verified = False
        for stage in RefinementStage:
            session.motion_states[stage] = (
                RefinementMotionState.NOT_TESTED
            )
        self._hand_depth_history.clear()
        self.refinement_recovery_status_label.setText(message)
        self._update_save_probe_point_state()

    def handle_refinement_emergency_stop(self):
        """Cancel robot motion while keeping recovery state explicit."""
        session = self._refinement_session
        if session is not None:
            pending = session.pending_motion
            inward_or_unknown_probe_motion = (
                session.recovery_required
                or (
                    pending is not None
                    and pending.stage == RefinementStage.PROBE
                )
            )
            if pending is not None:
                session.fail_motion(
                    pending.request_id,
                    "Emergency stop triggered",
                )
            if inward_or_unknown_probe_motion:
                session.require_recovery(
                    "Emergency stop triggered after probe-axis motion. "
                    "Verify clearance, then retract to the aligned "
                    "pre-approach pose."
                )
            else:
                self._clear_refinement_execution_evidence(
                    "Emergency stop triggered. Movement evidence was "
                    "cleared; re-establish the ordered workflow."
                )
        self._probe_motion_pending = False
        self.ui.handle_simple_command(CommandID.EMERGENCY_CANCEL)
        self._refresh_refinement_dialog()

    @staticmethod
    def _pose_errors(target, achieved):
        position_error = math.sqrt(
            (target.position.x - achieved.position.x) ** 2
            + (target.position.y - achieved.position.y) ** 2
            + (target.position.z - achieved.position.z) ** 2
        )
        dot = abs(
            target.orientation.x * achieved.orientation.x
            + target.orientation.y * achieved.orientation.y
            + target.orientation.z * achieved.orientation.z
            + target.orientation.w * achieved.orientation.w
        )
        orientation_error = 2.0 * math.acos(
            max(-1.0, min(1.0, dot))
        )
        return position_error, orientation_error

    def _require_command_path_idle(self, require_settled=False):
        if self._buffered_command_count:
            raise RuntimeError("Command buffer must be empty")
        if (
            self._probe_motion_pending
            or self._command_state == CommandStatus.STATE_RUNNING
        ):
            raise RuntimeError("Wait for the current robot command to finish")
        if require_settled:
            elapsed = (
                time.monotonic()
                - self._last_command_completion_monotonic
            )
            if elapsed < PROBE_MOTION_SETTLE_SEC:
                remaining = PROBE_MOTION_SETTLE_SEC - elapsed
                raise RuntimeError(
                    "Wait for the arm to settle for another "
                    f"{remaining:.2f} s"
                )

    def _measure_live_surface_distance_samples(self):
        if self.node is None:
            raise RuntimeError("ROS is unavailable in the inspection UI")
        camera_info = self._latest_hand_depth_camera_info
        if camera_info is None:
            raise ValueError("No live registered hand-depth sample is available")
        required_receipt_time = (
            self._last_command_completion_monotonic
            + PROBE_MOTION_SETTLE_SEC
        )
        now = self.node.get_clock().now()
        samples = []
        errors = []
        for _, receipt_time, depth_image in self._hand_depth_history:
            if receipt_time < required_receipt_time:
                continue
            try:
                stamp = Time.from_msg(depth_image.header.stamp)
                if stamp.nanoseconds <= 0:
                    raise ValueError("Live hand-depth timestamp is empty")
                age_seconds = (now - stamp).nanoseconds * 1e-9
                if age_seconds < -0.05:
                    raise ValueError(
                        "Live hand-depth timestamp is in the future"
                    )
                if age_seconds > MAX_LIVE_DEPTH_AGE_SEC:
                    continue
                depth_frame = (
                    depth_image.header.frame_id.strip()
                    or camera_info.header.frame_id.strip()
                )
                if not depth_frame:
                    raise ValueError("Live hand-depth frame is empty")
                probe_to_camera = self._lookup_pose(
                    self._active_probe_frame(),
                    depth_frame,
                    lookup_time=stamp,
                )
                samples.append(
                    measure_probe_surface_distance(
                        depth_image,
                        camera_info,
                        probe_to_camera,
                    )
                )
            except Exception as exception:
                errors.append(str(exception))
        if len(samples) < 3:
            detail = f" Last rejection: {errors[-1]}" if errors else ""
            raise ValueError(
                "Fewer than three valid post-settle depth frames are "
                f"available.{detail}"
            )
        return samples

    def _measure_live_surface_distance(self):
        samples = self._measure_live_surface_distance_samples()
        return samples[-1]

    def _build_probe_pose_command(self, probe_pose_object):
        probe_pose_object.validate()
        tag = self._live_reference_tag()
        body_to_object = self._pose_data_from_message(tag.pose.pose)
        hand_to_probe = self._hand_to_probe_pose()
        object_to_hand = probe_pose_to_hand_pose(
            probe_pose_object,
            hand_to_probe,
        )
        body_to_hand = compose_poses(body_to_object, object_to_hand)

        command = self._new_command(CommandID.MOVE_ARM_TO_TAG)
        command.tag = deepcopy(tag)
        command.offset.header = deepcopy(tag.pose.header)
        command.offset.header.frame_id = tag.pose.header.frame_id
        command.offset.pose.position.x = (
            body_to_hand.position.x - body_to_object.position.x
        )
        command.offset.pose.position.y = (
            body_to_hand.position.y - body_to_object.position.y
        )
        command.offset.pose.position.z = (
            body_to_hand.position.z - body_to_object.position.z
        )
        self._write_quaternion_message(
            command.offset.pose.orientation,
            body_to_hand.orientation,
        )
        command.orientation_mode = OrientationModes.CUSTOM_ORIENTATION.value
        return command

    def _current_probe_pose_object(self):
        tag = self._live_reference_tag()
        body_frame = tag.pose.header.frame_id.strip()
        probe_frame = self._active_probe_frame()
        body_to_probe = self._lookup_pose(body_frame, probe_frame)
        body_to_object = self._pose_data_from_message(tag.pose.pose)
        return relative_pose(body_to_object, body_to_probe)

    def _hand_to_probe_pose(self):
        probe_frame = self._active_probe_frame()
        if probe_frame == HAND_FRAME_NAME:
            return PoseData.identity()
        return self._lookup_pose(HAND_FRAME_NAME, probe_frame)

    def _active_probe_frame(self):
        routine_id = self.saved_routine_dropdown.currentData()
        if routine_id and self._selected_definition is not None:
            routine = self._selected_definition.get_routine(routine_id)
            if routine is not None:
                return sensor_probe_frame(routine.sensor_id)

        sensor_id = self.sensor_id_field.currentData()
        if not sensor_id:
            raise ValueError("Sensor mounting must be selected")
        return sensor_probe_frame(sensor_id)

    def _lookup_pose(
        self,
        target_frame,
        source_frame,
        lookup_time=None,
    ):
        if self._tf_buffer is None:
            raise RuntimeError("TF is unavailable in the inspection UI")
        transform = self._tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            lookup_time or Time(),
            timeout=Duration(seconds=0.5),
        )
        value = transform.transform
        pose = PoseData(
            position=Vector3Data(
                x=value.translation.x,
                y=value.translation.y,
                z=value.translation.z,
            ),
            orientation=QuaternionData(
                x=value.rotation.x,
                y=value.rotation.y,
                z=value.rotation.z,
                w=value.rotation.w,
            ),
        )
        pose.validate()
        return pose

    def _live_reference_tag(self):
        if self._selected_definition is None:
            raise ValueError("No inspection object is selected")
        tag_id = self._selected_definition.reference_tag.tag_id
        if self.node is not None:
            return self._stable_reference_tag(tag_id)
        base_tags = getattr(self.ui, "base_tags", None)
        if base_tags is None:
            base_tags = getattr(self.ui, "visible_tags", {})
        tag = base_tags.get(tag_id)
        if tag is None:
            raise ValueError(
                f"Base-camera reference tag {tag_id} must be visible"
            )
        if not tag.pose.header.frame_id.strip():
            raise ValueError("Reference tag pose frame is empty")
        return tag

    def _stable_reference_tag(self, tag_id):
        history = self._base_tag_histories.get(tag_id, ())
        samples = []
        messages_by_stamp = {}
        for stamp_key, tag in history:
            stamp_seconds = (
                float(stamp_key[0]) + float(stamp_key[1]) * 1e-9
            )
            samples.append(
                TagPoseSample(
                    stamp_seconds=stamp_seconds,
                    frame_id=tag.pose.header.frame_id.strip(),
                    pose=self._pose_data_from_message(tag.pose.pose),
                )
            )
            messages_by_stamp[stamp_seconds] = tag
        now_seconds = self.node.get_clock().now().nanoseconds * 1e-9
        stable = stabilize_tag_pose(
            samples,
            now_seconds=now_seconds,
            maximum_age_sec=BASE_TAG_MAXIMUM_AGE_SEC,
            stabilization_window_sec=BASE_TAG_STABILIZATION_HISTORY_SEC,
            minimum_samples=3,
            minimum_span_sec=BASE_TAG_MINIMUM_SPAN_SEC,
        )
        tag = deepcopy(messages_by_stamp[stable.newest_stamp_seconds])
        tag.pose.header.frame_id = stable.frame_id
        tag.pose.pose.position.x = stable.pose.position.x
        tag.pose.pose.position.y = stable.pose.position.y
        tag.pose.pose.position.z = stable.pose.position.z
        self._write_quaternion_message(
            tag.pose.pose.orientation,
            stable.pose.orientation,
        )
        return tag

    def _require_probe_setup(self):
        if self._probe_setup is None:
            raise ValueError("No calculated probe setup is available")
        return self._probe_setup

    def _show_setup_error(self, title, exception):
        self._set_setup_status("Unavailable", str(exception))
        self.show_warning(title, str(exception))

    def _set_status_text(self, text):
        if self.status_label is not None:
            self.status_label.setText(text)

    @staticmethod
    def _pose_data_from_message(pose):
        result = PoseData(
            position=Vector3Data(
                x=pose.position.x,
                y=pose.position.y,
                z=pose.position.z,
            ),
            orientation=QuaternionData(
                x=pose.orientation.x,
                y=pose.orientation.y,
                z=pose.orientation.z,
                w=pose.orientation.w,
            ),
        )
        result.validate()
        return result

    @staticmethod
    def _write_quaternion_message(message, quaternion):
        quaternion.validate()
        message.x = quaternion.x
        message.y = quaternion.y
        message.z = quaternion.z
        message.w = quaternion.w

    @staticmethod
    def _distance_value(field, label):
        text = field.text().strip()
        try:
            value = float(text)
        except ValueError as exception:
            raise ValueError(f"{label} must be a number") from exception
        if value <= 0.0:
            raise ValueError(f"{label} must be positive")
        return value

    @staticmethod
    def _bounded_positive_value(field, label, maximum):
        value = InspectionControls._distance_value(field, label)
        if value > maximum:
            raise ValueError(f"{label} must not exceed {maximum:g}")
        return value

    def _required_text(self, field, label):
        value = field.text().strip()
        if value:
            return value
        self.show_warning("Missing Input", f"Enter {label}.")
        return None

    def _new_command(self, command_id):
        command = ComplexCommand()
        command.command = self.ui.build_basic_command(command_id)
        if not command.command.request_id:
            command.command.request_id = new_request_id()
        return command

    def _publish(self, command):
        self.complex_command_publisher.publish(command)
        self.status_label.setText(
            f"Command sent: {command.command.command_id}"
        )
        self._schedule_repository_refresh()

    def refresh_saved_definitions(
        self,
        desired_object_id=None,
        desired_routine_id=None,
    ):
        """Reload definitions and select only existing repository entries."""
        object_ids = self.object_repository.list_object_ids()
        current_object_id = (
            self.saved_object_dropdown.currentData() or ""
        )

        if desired_object_id is None:
            target_object_id = (
                current_object_id
                if current_object_id in object_ids
                else ""
            )
        elif desired_object_id in object_ids:
            target_object_id = desired_object_id
        else:
            target_object_id = ""

        self.saved_object_dropdown.blockSignals(True)
        self.saved_object_dropdown.clear()
        self.saved_object_dropdown.addItem(
            "Select saved object",
            None,
        )
        for object_id in object_ids:
            self.saved_object_dropdown.addItem(object_id, object_id)

        selected_index = self.saved_object_dropdown.findData(
            target_object_id
        )
        self.saved_object_dropdown.setCurrentIndex(
            selected_index if selected_index >= 0 else 0
        )
        self.saved_object_dropdown.blockSignals(False)

        self._refresh_routine_parent_objects(
            object_ids,
            target_object_id,
        )
        self._load_selected_object(
            desired_routine_id=desired_routine_id,
        )

    def _refresh_routine_parent_objects(
        self,
        object_ids=None,
        desired_object_id="",
    ):
        if not hasattr(self, "routine_parent_object_dropdown"):
            return
        if object_ids is None:
            object_ids = self.object_repository.list_object_ids()

        current_parent_id = (
            self.routine_parent_object_dropdown.currentData() or ""
        )
        if desired_object_id not in object_ids:
            desired_object_id = (
                current_parent_id
                if current_parent_id in object_ids
                else ""
            )
        if not desired_object_id and len(object_ids) == 1:
            desired_object_id = object_ids[0]

        dropdown = self.routine_parent_object_dropdown
        dropdown.blockSignals(True)
        dropdown.clear()
        dropdown.addItem("Select existing object", None)
        for object_id in object_ids:
            dropdown.addItem(object_id, object_id)
        selected_index = dropdown.findData(desired_object_id)
        dropdown.setCurrentIndex(
            selected_index if selected_index >= 0 else 0
        )
        dropdown.blockSignals(False)
        selected_sensor_id = self.sensor_id_field.currentData()
        self.create_routine_button.setEnabled(
            bool(object_ids)
            and selected_sensor_id in self._sensor_definitions
        )

    def _load_selected_object(
        self,
        _index=None,
        desired_routine_id=None,
    ):
        object_id = self.saved_object_dropdown.currentData()
        self.delete_object_button.setEnabled(bool(object_id))
        self.delete_routine_button.setEnabled(False)
        if not object_id:
            self._selected_definition = None
            self._populate_routine_dropdown([])
            self.reference_view_status_label.setText(
                "Reference view: no routine selected"
            )
            self._clear_reference_previews(
                "No reference view selected"
            )
            return

        try:
            definition = self.object_repository.load(object_id)
        except Exception as exception:
            self._selected_definition = None
            self._populate_routine_dropdown([])
            self.reference_view_status_label.setText(
                f"Definition load failed: {exception}"
            )
            self._clear_reference_previews(
                "Reference view unavailable"
            )
            return

        if desired_routine_id is None:
            desired_routine_id = (
                self.saved_routine_dropdown.currentData() or ""
            )

        self._selected_definition = definition
        if not self.management_dialog.isVisible():
            self.object_id_field.setText(definition.object_id)
            self.object_display_name_field.setText(
                definition.display_name
            )
            self.reference_tag_id_field.setText(
                str(definition.reference_tag.tag_id)
            )
            self.reference_tag_family_field.setText(
                definition.reference_tag.tag_family
            )
        parent_index = self.routine_parent_object_dropdown.findData(
            definition.object_id
        )
        if parent_index >= 0:
            self.routine_parent_object_dropdown.setCurrentIndex(
                parent_index
            )
        self._populate_routine_dropdown(
            definition.routines,
            desired_routine_id,
        )

    def _populate_routine_dropdown(
        self,
        routines,
        desired_routine_id="",
    ):
        self.saved_routine_dropdown.blockSignals(True)
        self.saved_routine_dropdown.clear()
        self.saved_routine_dropdown.addItem(
            "Select saved routine",
            None,
        )
        for routine in sorted(
            routines,
            key=lambda candidate: candidate.routine_id,
        ):
            self.saved_routine_dropdown.addItem(
                routine.routine_id,
                routine.routine_id,
            )
        selected_index = self.saved_routine_dropdown.findData(
            desired_routine_id
        )
        self.saved_routine_dropdown.setCurrentIndex(
            selected_index if selected_index >= 0 else 0
        )
        self.saved_routine_dropdown.blockSignals(False)
        self._load_selected_routine()

    def _load_selected_routine(self, _index=None):
        self._clear_reference_previews("No reference view selected")
        routine_id = self.saved_routine_dropdown.currentData()
        self.delete_routine_button.setEnabled(
            bool(routine_id and self._selected_definition is not None)
        )
        if not routine_id or self._selected_definition is None:
            self.reference_view_status_label.setText(
                "Reference view: no routine selected"
            )
            self.reference_view_status_label.setToolTip("")
            return

        routine = self._selected_definition.get_routine(routine_id)
        if routine is None:
            self.reference_view_status_label.setText(
                "Reference view: routine no longer exists"
            )
            self.reference_view_status_label.setToolTip("")
            self._clear_reference_previews(
                "Reference view unavailable"
            )
            return

        if not self.management_dialog.isVisible():
            self.routine_id_field.setText(routine.routine_id)
            self.routine_display_name_field.setText(
                routine.display_name
            )
            self._populate_sensor_dropdown(routine.sensor_id)

        if not routine.reference_views:
            self._set_reference_camera_defaults()
            self.reference_view_status_label.setText(
                "Reference view: not captured"
            )
            self.reference_view_status_label.setToolTip("")
            self._clear_reference_previews(
                "No reference view captured"
            )
            return

        self._clear_reference_camera_selections()
        self._reference_slot_captures = [None, None, None]
        camera_names = []
        unavailable = []
        object_id = self._selected_definition.object_id
        reference_tag_id = self._selected_definition.reference_tag.tag_id

        for reference_view in sorted(
            routine.reference_views,
            key=lambda view: view.slot_index,
        ):
            slot_index = reference_view.slot_index
            camera_id = reference_view.camera_id
            if slot_index < 0 or slot_index > 2:
                unavailable.append(
                    f"Invalid slot {slot_index + 1}: {camera_id}"
                )
                continue
            camera_config = REFERENCE_CAMERA_BY_ID.get(camera_id)
            widget = self.reference_view_widgets[slot_index]
            if camera_config is None:
                unavailable.append(
                    f"Slot {slot_index + 1}: unknown camera {camera_id}"
                )
                widget.clear_preview("Unknown camera")
                continue

            dropdown = self.reference_camera_dropdowns[slot_index]
            dropdown.blockSignals(True)
            dropdown.setCurrentIndex(dropdown.findData(camera_id))
            dropdown.blockSignals(False)

            try:
                capture = self._load_reference_view_capture(
                    object_id,
                    routine.routine_id,
                    reference_tag_id,
                    reference_view,
                )
                valid_region = rgb_depth_selectable_region(
                    (
                        capture.rgb_image.width,
                        capture.rgb_image.height,
                    ),
                    capture.depth_image,
                    capture.rgb_camera_info,
                    capture.depth_camera_info,
                )
            except Exception as exception:
                detail = (
                    f"Slot {slot_index + 1} {camera_config.display_name}: "
                    f"{exception}"
                )
                unavailable.append(detail)
                widget.clear_preview(
                    f"{camera_config.display_name} unavailable"
                )
                widget.setToolTip(detail)
                continue

            self._reference_slot_captures[slot_index] = capture
            widget.setToolTip("")
            widget.set_ros_image(
                capture.rgb_image,
                valid_region=valid_region,
            )
            camera_names.append(camera_config.display_name)

        for slot_index, capture in enumerate(
            self._reference_slot_captures
        ):
            if capture is not None:
                continue
            camera_id = (
                self.reference_camera_dropdowns[slot_index].currentData()
                or ""
            )
            widget = self.reference_view_widgets[slot_index]
            if camera_id and not widget.text():
                widget.clear_preview(
                    f"Capture {REFERENCE_CAMERA_BY_ID[camera_id].display_name}"
                )
            elif not camera_id:
                widget.clear_preview("No camera selected")

        captures = [
            capture
            for capture in self._reference_slot_captures
            if capture is not None
        ]
        if not captures:
            self._reference_rgb_size = None
            self._reference_depth_image = None
            self._reference_rgb_camera_info = None
            self._reference_camera_info = None
            self._reference_view = None
            for reference_view in routine.reference_views:
                if 0 <= reference_view.slot_index <= 2:
                    self.reference_view_widgets[
                        reference_view.slot_index
                    ].clear_preview("Reference view unavailable")
            dataset_paths = [
                reference_view.reference_dataset_path
                or reference_view.view_id
                or f"slot{reference_view.slot_index + 1}"
                for reference_view in sorted(
                    routine.reference_views,
                    key=lambda view: view.slot_index,
                )
            ]
            self.reference_view_status_label.setText(
                "Reference view: captured "
                f"({', '.join(dataset_paths)}); preview unavailable"
            )
            self.reference_view_status_label.setToolTip(
                "\n".join(unavailable)
            )
            return

        first_capture = captures[0]
        self._reference_rgb_size = (
            first_capture.rgb_image.width,
            first_capture.rgb_image.height,
        )
        self._reference_depth_image = first_capture.depth_image
        self._reference_rgb_camera_info = (
            first_capture.rgb_camera_info
        )
        self._reference_camera_info = (
            first_capture.depth_camera_info
        )
        self._reference_view = first_capture.reference_view

        status = "Reference view: captured " + ", ".join(camera_names)
        if unavailable:
            status += f"; {len(unavailable)} unavailable"
        self.reference_view_status_label.setText(status)
        self.reference_view_status_label.setToolTip(
            "\n".join(unavailable)
        )

    def _load_reference_view_capture(
        self,
        object_id,
        routine_id,
        reference_tag_id,
        reference_view,
    ):
        repository = self.reference_view_repository
        dataset_path = repository._dataset_path(
            object_id,
            routine_id,
            reference_view,
        )
        metadata = json.loads(
            (dataset_path / "metadata.json").read_text(
                encoding="utf-8"
            )
        )
        repository._validate_metadata(
            metadata,
            object_id,
            routine_id,
            reference_view,
        )
        rgb_image = deserialize_message(
            (dataset_path / "rgb.cdr").read_bytes(),
            Image,
        )
        depth_image = deserialize_message(
            (dataset_path / "depth.cdr").read_bytes(),
            Image,
        )
        rgb_camera_info = deserialize_message(
            (dataset_path / "rgb_camera_info.cdr").read_bytes(),
            CameraInfo,
        )
        depth_camera_info = deserialize_message(
            (dataset_path / "depth_camera_info.cdr").read_bytes(),
            CameraInfo,
        )
        reference_tag = deserialize_message(
            (dataset_path / "reference_tag.cdr").read_bytes(),
            TagElement,
        )
        repository._validate_loaded_dataset(
            metadata,
            reference_view,
            reference_tag_id,
            rgb_image,
            depth_image,
            rgb_camera_info,
            depth_camera_info,
            reference_tag,
        )
        return CapturedReferenceView(
            slot_index=reference_view.slot_index,
            camera_id=reference_view.camera_id,
            reference_view=reference_view,
            rgb_image=rgb_image,
            depth_image=depth_image,
            rgb_camera_info=rgb_camera_info,
            depth_camera_info=depth_camera_info,
            reference_tag=reference_tag,
            fixed_frame=str(metadata["fixed_frame"]),
        )

    def _schedule_repository_refresh(self):
        for delay_ms in (250, 1000, 3500):
            QTimer.singleShot(delay_ms, self.refresh_saved_definitions)

    def _publish_management_compatibility_command(self, command):
        if self.node is None:
            self.complex_command_publisher.publish(command)

    def handle_create_object(self):
        """Create an inspection object directly in the repository."""
        object_id = self._required_text(
            self.object_id_field,
            "an object ID",
        )
        display_name = self._required_text(
            self.object_display_name_field,
            "an object display name",
        )
        tag_id_text = self._required_text(
            self.reference_tag_id_field,
            "a reference tag ID",
        )
        tag_family = self._required_text(
            self.reference_tag_family_field,
            "a reference tag family",
        )
        if None in (object_id, display_name, tag_id_text, tag_family):
            return False

        try:
            tag_id = int(tag_id_text)
            if tag_id < 0:
                raise ValueError
        except ValueError:
            self.show_warning(
                "Invalid Input",
                "Reference tag ID must be a non-negative integer.",
            )
            return False

        definition = InspectionObject(
            object_id=object_id,
            display_name=display_name,
            reference_tag=ReferenceTag(
                tag_id=tag_id,
                tag_family=tag_family,
            ),
        )
        try:
            self.object_repository.create(definition)
        except Exception as exception:
            self.management_status_label.setText(
                f"Object creation failed: {exception}"
            )
            self.show_warning("Create Inspection Object", str(exception))
            self.refresh_saved_definitions()
            return False

        command = self._new_command(CommandID.CREATE_INSPECTION_OBJECT)
        command.inspection.object.object_id = object_id
        command.inspection.object.display_name = display_name
        command.inspection.object.reference_tag_id = tag_id
        command.inspection.object.reference_tag_family = tag_family
        self._publish_management_compatibility_command(command)

        self.refresh_saved_definitions(
            desired_object_id=object_id,
            desired_routine_id="",
        )
        self.object_id_field.clear()
        self.object_display_name_field.clear()
        self.reference_tag_id_field.clear()
        self.reference_tag_family_field.setText("36h11")
        self.management_status_label.setText(
            f"Created inspection object '{object_id}'."
        )
        self._set_status_text(
            f"Created inspection object '{object_id}'"
        )
        return True

    def handle_create_routine(self):
        """Create a routine directly under the selected existing object."""
        object_id = self.routine_parent_object_dropdown.currentData()
        if not object_id and self.node is None:
            object_id = self.object_id_field.text().strip()
        if not object_id:
            self.show_warning(
                "No Parent Object Selected",
                "Select an existing inspection object.",
            )
            return False

        routine_id = self._required_text(
            self.routine_id_field,
            "a routine ID",
        )
        display_name = self._required_text(
            self.routine_display_name_field,
            "a routine display name",
        )
        sensor_id = self.sensor_id_field.currentData()
        if sensor_id not in self._sensor_definitions:
            self.show_warning(
                "No Sensor Selected",
                "Select a registered sensor mounting.",
            )
            return False
        if None in (
            routine_id,
            display_name,
            sensor_id,
        ):
            return False

        routine = InspectionRoutine(
            routine_id=routine_id,
            display_name=display_name,
            sensor_id=sensor_id,
        )
        direct_creation_succeeded = False
        try:
            self.object_repository.add_routine(object_id, routine)
            direct_creation_succeeded = True
        except FileNotFoundError as exception:
            if self.node is not None:
                self.management_status_label.setText(
                    f"Routine creation failed: {exception}"
                )
                self.show_warning(
                    "Create Inspection Routine",
                    str(exception),
                )
                self.refresh_saved_definitions()
                return False
        except Exception as exception:
            self.management_status_label.setText(
                f"Routine creation failed: {exception}"
            )
            self.show_warning("Create Inspection Routine", str(exception))
            self.refresh_saved_definitions(
                desired_object_id=object_id,
            )
            return False

        command = self._new_command(CommandID.CREATE_INSPECTION_ROUTINE)
        command.inspection.object.object_id = object_id
        command.inspection.routine.routine_id = routine_id
        command.inspection.routine.display_name = display_name
        command.inspection.routine.sensor_id = sensor_id
        self._publish_management_compatibility_command(command)

        if direct_creation_succeeded:
            self.refresh_saved_definitions(
                desired_object_id=object_id,
                desired_routine_id=routine_id,
            )
        self.routine_id_field.clear()
        self.routine_display_name_field.clear()
        self.management_status_label.setText(
            f"Created inspection routine '{object_id}/{routine_id}'."
        )
        self._set_status_text(
            f"Created inspection routine '{object_id}/{routine_id}'"
        )
        return True

    def handle_capture_reference_view(self):
        """Publish one capture request for up to three camera slots."""
        object_id = self.saved_object_dropdown.currentData()
        routine_id = self.saved_routine_dropdown.currentData()
        if self.node is None:
            object_id = object_id or self.object_id_field.text().strip()
            routine_id = routine_id or self.routine_id_field.text().strip()
        if not object_id:
            self.show_warning(
                "No Object Selected",
                "Select a saved inspection object.",
            )
            return False
        if not routine_id:
            self.show_warning(
                "No Routine Selected",
                "Select a saved inspection routine.",
            )
            return False

        camera_ids = self._selected_reference_camera_ids()
        selected = [camera_id for camera_id in camera_ids if camera_id]
        if not selected:
            self.show_warning(
                "No Camera Selected",
                "Select at least one reference camera.",
            )
            return False
        if len(selected) != len(set(selected)):
            self.show_warning(
                "Duplicate Camera",
                "Each reference camera can only be selected once.",
            )
            return False

        replace_existing = self.replace_reference_view_checkbox.isChecked()
        if replace_existing and not self.ask_question(
            "Replace Reference Views",
            (
                f"Replace all saved reference views for "
                f"'{object_id}/{routine_id}'?"
            ),
        ):
            return False

        command = self._new_command(
            CommandID.CAPTURE_INSPECTION_OBJECT_REFERENCE_VIEW
        )
        command.inspection.object.object_id = object_id
        command.inspection.routine.routine_id = routine_id
        command.inspection.replace_existing = replace_existing
        command.inspection.reference_camera_ids = camera_ids
        self._publish(command)
        return True

    def handle_delete_object(self):
        """Delete the selected object and immediately rebuild the UI."""
        object_id = self.saved_object_dropdown.currentData()
        if not object_id:
            self.show_warning(
                "No Object Selected",
                "Select a saved inspection object to delete.",
            )
            return False
        if not self.ask_question(
            "Delete Inspection Object",
            (
                f"Delete '{object_id}' and all of its routines, "
                "reference views, and probe points?"
            ),
        ):
            return False

        try:
            deleted = self.object_repository.delete_object(object_id)
            if not deleted:
                raise FileNotFoundError(
                    f"Inspection object does not exist: {object_id}"
                )
        except Exception as exception:
            self.show_warning("Delete Inspection Object", str(exception))
            self.refresh_saved_definitions()
            return False

        command = self._new_command(
            CommandID.DELETE_INSPECTION_OBJECT
        )
        command.inspection.object.object_id = object_id
        self._publish_management_compatibility_command(command)

        self.refresh_saved_definitions(
            desired_object_id="",
            desired_routine_id="",
        )
        self.management_status_label.setText(
            f"Deleted inspection object '{object_id}'."
        )
        self._set_status_text(
            f"Deleted inspection object '{object_id}'"
        )
        return True

    def handle_delete_routine(self):
        """Delete the selected routine and immediately rebuild the UI."""
        object_id = self.saved_object_dropdown.currentData()
        routine_id = self.saved_routine_dropdown.currentData()
        if not object_id:
            self.show_warning(
                "No Object Selected",
                "Select a saved inspection object first.",
            )
            return False
        if not routine_id:
            self.show_warning(
                "No Routine Selected",
                "Select a saved inspection routine to delete.",
            )
            return False
        if not self.ask_question(
            "Delete Inspection Routine",
            (
                f"Delete '{object_id}/{routine_id}' including its "
                "reference view and probe points?"
            ),
        ):
            return False

        try:
            self.object_repository.delete_routine(
                object_id,
                routine_id,
            )
        except Exception as exception:
            self.show_warning("Delete Inspection Routine", str(exception))
            self.refresh_saved_definitions(
                desired_object_id=object_id,
            )
            return False

        command = self._new_command(
            CommandID.DELETE_INSPECTION_ROUTINE
        )
        command.inspection.object.object_id = object_id
        command.inspection.routine.routine_id = routine_id
        self._publish_management_compatibility_command(command)

        self.refresh_saved_definitions(
            desired_object_id=object_id,
            desired_routine_id="",
        )
        self.management_status_label.setText(
            f"Deleted inspection routine '{object_id}/{routine_id}'."
        )
        self._set_status_text(
            f"Deleted inspection routine '{object_id}/{routine_id}'"
        )
        return True
