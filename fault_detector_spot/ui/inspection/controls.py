"""Inspection setup controls."""

import math
from copy import deepcopy

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
    ProbeSetupIntent,
    ProbeSetupMotionIntent,
    ProbeSetupState,
    SensorDefinitionArray,
)
from fault_detector_msgs.srv import AddSensor, RetireSensor

from fault_detector_spot.shared.ros.qos_profiles import LATCHED_QOS
from fault_detector_spot.inspection.model.models import (
    ImagePoint,
    MINIMUM_ALIGNED_PREAPPROACH_SEPARATION_M,
    PoseData,
    QuaternionData,
    Vector3Data,
)
from fault_detector_spot.inspection.setup.probe_refinement_session import (
    RefinementMotionState,
    RefinementStage,
)
from fault_detector_spot.inspection.setup.reference_camera_registry import (
    REFERENCE_CAMERAS,
    REFERENCE_CAMERA_BY_ID,
)
from fault_detector_spot.inspection.setup.reference_view_approach_direction import (
    APPROACH_MODE_AUTOMATIC,
    APPROACH_MODE_SURFACE_FIT,
    APPROACH_MODE_TAG_X,
    APPROACH_SOURCE_SURFACE_FIT,
    APPROACH_SOURCE_TAG_X_SELECTED,
)
from fault_detector_spot.inspection.setup.reference_view_depth_projection import (
    ImageRegion,
)
from fault_detector_spot.inspection.setup.reference_probe_setup import (
    probe_pose_to_hand_pose,
)
from fault_detector_spot.inspection.setup.reference_view_surface_target import (
    quaternion_to_rpy,
)
from fault_detector_spot.inspection.model.sensor_models import (
    SensorDefinition,
    sensor_definition_from_values,
    sensor_probe_frame,
)

from .probe_refinement_dialog import ProbeRefinementDialog
from .reference_view_widget import ReferenceViewWidget
from ..ros.probe_setup_state_adapter import probe_setup_state_to_view
from ..shared.collapsible_section import CollapsibleSection
from ..shared.control_helper import UIControlHelper


MAX_REFINEMENT_TRANSLATION_M = 0.05
MAX_REFINEMENT_ROTATION_DEG = 15.0
SURFACE_DISTANCE_TOLERANCE_M = 0.005
REFINEMENT_FRAME_SENSOR = "sensor"
REFINEMENT_FRAME_HAND = "hand"
REFINEMENT_FRAME_TAG = "tag"
REFINEMENT_FRAME_BODY = "body"
REFINEMENT_FRAME_MAP = "map"


class InspectionControls(UIControlHelper):
    """Render probe setup state and submit typed user intent."""

    def __init__(self, parent_ui):
        """Create controls backed by the remote probe setup API."""
        self._probe_setup_state = None
        self._selected_sensor_id = ""
        self._reference_rgb_size = None
        self._reference_depth_image = None
        self._reference_rgb_camera_info = None
        self._reference_camera_info = None
        self._reference_view = None
        self._reference_slot_view_ids = ["", "", ""]
        self._preview_signature = None
        self._active_reference_slot = None
        self._selected_surface_point = None
        self._selected_surface_normal = None
        self._surface_normal_error = ""
        self._selected_approach_direction = None
        self._selected_surface_target = None
        self._calculated_surface_probe_orientation = None
        self._calculated_surface_hand_orientation = None
        self._calculated_probe_setup = None
        self._probe_setup = None
        self._refinement_presentation = None
        self._sensor_definitions = {}
        self._pending_sensor_selection = ""
        self._pending_sensor_retirement = ""
        self._distance_failure_requires_retraction = False
        self._retraction_failed = False
        self._editing_probe_point_id = None
        self.sensor_add_client = None
        self.sensor_retire_client = None
        self.sensor_list_subscription = None
        self.management_dialog = None
        super().__init__(parent_ui)
        initial_sensors = getattr(parent_ui, "sensor_definitions", [])
        if initial_sensors:
            self.set_sensor_definitions(initial_sensors)
        self.refresh_setup_state()

    def add_rows(self, layout):
        """Add the rows constructed during initialization."""
        for row in self.rows:
            layout.addLayout(row)

    def init_ros_communication(self):
        """Create presentation-side sensor registry transport."""
        if self.node is None:
            return
        self.sensor_add_client = self.node.create_client(
            AddSensor,
            "fault_detector/add_sensor",
        )
        self.sensor_retire_client = self.node.create_client(
            RetireSensor,
            "fault_detector/retire_sensor",
        )
        self.sensor_list_subscription = self.node.create_subscription(
            SensorDefinitionArray,
            "fault_detector/sensors",
            self._process_sensor_definitions,
            LATCHED_QOS,
        )
        client = getattr(self.ui, "probe_setup_client", None)
        if client is not None:
            client.surface_orientation_received.connect(
                self.apply_live_surface_orientation
            )
            client.surface_orientation_rejected.connect(
                self.apply_live_surface_orientation_error
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
            "Storage: managed by the application service"
        )
        self.storage_path_label.setTextInteractionFlags(
            Qt.TextSelectableByMouse
        )
        dialog_layout.addWidget(self.storage_path_label)

        self.management_status_label = QLabel(
            "Create and delete operations are server-owned transactions."
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


    def handle_application_state(self, _status):
        """Keep the top-level application-state callback presentation-only."""
        return None

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
            self.handle_clear_reference_pixel
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
        self.alignment_orientation_mode_dropdown = QComboBox()
        self.alignment_orientation_mode_dropdown.addItem(
            "Align with tag",
            ProbeSetupMotionIntent.ALIGNMENT_ORIENTATION_TAG,
        )
        self.alignment_orientation_mode_dropdown.addItem(
            "Align with calculated surface",
            ProbeSetupMotionIntent.ALIGNMENT_ORIENTATION_CALCULATED_SURFACE,
        )
        self.calculated_surface_orientation_value_label = QLabel(
            "Not calculated"
        )
        self.calculated_surface_orientation_value_label.setWordWrap(True)
        self.calculated_surface_orientation_value_label.setTextInteractionFlags(
            Qt.TextSelectableByMouse
        )
        self.calculate_hand_surface_orientation_button = QPushButton(
            "Calculate Hand-Facing Surface"
        )
        self.orient_to_calculated_surface_button = QPushButton(
            "Orient to Calculated Surface"
        )
        self.orient_to_calculated_surface_button.setEnabled(False)
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
        self.surface_distance_tolerance_field.setReadOnly(True)
        self.surface_distance_tolerance_field.setToolTip(
            "Server-owned surface verification policy"
        )
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
        self.alignment_orientation_mode_dropdown.currentIndexChanged.connect(
            self._handle_alignment_orientation_mode_changed
        )
        self.calculate_hand_surface_orientation_button.clicked.connect(
            self.handle_calculate_hand_surface_orientation
        )
        self.orient_to_calculated_surface_button.clicked.connect(
            self.handle_orient_to_calculated_surface
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
            QLabel("Commanded hand angle relative to object [deg]:"),
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

        if stage == "alignment":
            orientation_group = QGroupBox("Alignment orientation")
            orientation_layout = QFormLayout(orientation_group)
            orientation_layout.addRow(
                "Mode:",
                self.alignment_orientation_mode_dropdown,
            )
            orientation_layout.addRow(
                "Calculated hand orientation:",
                self.calculated_surface_orientation_value_label,
            )
            orientation_actions = QHBoxLayout()
            orientation_actions.addWidget(
                self.calculate_hand_surface_orientation_button
            )
            orientation_actions.addWidget(
                self.orient_to_calculated_surface_button
            )
            orientation_actions.addStretch()
            orientation_layout.addRow(orientation_actions)
            layout.addWidget(orientation_group)

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
            self.calculate_hand_surface_orientation_button,
            self.orient_to_calculated_surface_button,
            self.test_surface_distance_button,
            self.approve_and_retract_button,
            self.retract_without_saving_button,
        ):
            button.setEnabled(False)
        for stage_buttons in self.refinement_buttons.values():
            for button in stage_buttons.values():
                button.setEnabled(False)
        if self._refinement_presentation is not None:
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
        state = self._probe_setup_state
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
        duplicate = (
            state is not None
            and point_id in state.probe_point_ids
            and point_id != self._editing_probe_point_id
        )
        self.approve_and_retract_button.setEnabled(False)
        if state is None or not state.selected_routine_id:
            status = "Select a saved object and routine."
        elif not state.has_reference_pixel:
            status = "Select a point in a captured reference view."
        elif not point_id or not display_name:
            status = "Enter a probe point ID and display name."
        elif duplicate:
            status = f"Probe point '{point_id}' already exists."
        elif not numeric_ready:
            status = "Enter positive tolerances and measurement duration."
        else:
            status = (
                "Probe save and mandatory retraction await the "
                "coordinated execution workflow."
            )
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

    @staticmethod
    def _normalize_degrees(value):
        return (value + 180.0) % 360.0 - 180.0

    def _handle_reference_slot_point_changed(self, slot_index, u, v):
        view_id = self._reference_slot_view_ids[slot_index]
        for candidate_index, widget in enumerate(
            self.reference_view_widgets
        ):
            if candidate_index == slot_index:
                continue
            widget.blockSignals(True)
            widget.clear_selection()
            widget.blockSignals(False)
        self._active_reference_slot = slot_index
        if not view_id:
            self._show_setup_error(
                "Select Reference Point",
                ValueError("The selected preview has no reference view ID"),
            )
            return
        self._handle_reference_image_point_changed(u, v)
        intent = self._geometry_intent(
            ProbeSetupIntent.OPERATION_SELECT_REFERENCE_PIXEL
        )
        intent.reference_view_id = view_id
        intent.pixel_u = int(u)
        intent.pixel_v = int(v)
        self._submit_probe_setup(intent)

    def _handle_reference_slot_point_cleared(self, slot_index):
        if self._active_reference_slot == slot_index:
            self._handle_reference_image_point_cleared()
            self._active_reference_slot = None
            intent = ProbeSetupIntent()
            intent.operation = (
                ProbeSetupIntent.OPERATION_CLEAR_REFERENCE_PIXEL
            )
            self._submit_probe_setup(intent)

    def _clear_all_reference_selections(self):
        for widget in self.reference_view_widgets:
            widget.blockSignals(True)
            widget.clear_selection()
            widget.blockSignals(False)
        self._active_reference_slot = None
        self._handle_reference_image_point_cleared()

    def handle_clear_reference_pixel(self):
        self._clear_all_reference_selections()
        intent = ProbeSetupIntent()
        intent.operation = ProbeSetupIntent.OPERATION_CLEAR_REFERENCE_PIXEL
        return self._submit_probe_setup(intent)

    def _handle_reference_camera_selection_changed(self, slot_index):
        if slot_index >= len(self.reference_camera_dropdowns):
            return
        camera_id = (
            self.reference_camera_dropdowns[slot_index].currentData() or ""
        )
        self._reference_slot_view_ids[slot_index] = ""
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

    def _render_surface_verification_state(self, state):
        if not hasattr(self, "surface_distance_test_status_label"):
            return
        if state.surface_distance_tolerance_m > 0.0:
            self.surface_distance_tolerance_field.setText(
                f"{state.surface_distance_tolerance_m:.3f}"
            )
        if state.has_surface_distance_measurement:
            self.live_surface_distance_value_label.setText(
                self._format_readout_value(
                    state.measured_surface_distance_m,
                    4,
                )
            )
            self.surface_distance_delta_value_label.setText(
                self._format_readout_value(
                    state.surface_distance_error_m,
                    4,
                )
            )
        if not state.surface_verification_request_id:
            return
        labels = {
            ProbeSetupState.SURFACE_VERIFICATION_SAMPLING: "Sampling",
            ProbeSetupState.SURFACE_VERIFICATION_MOVING: "Moving",
            ProbeSetupState.SURFACE_VERIFICATION_SETTLING: "Settling",
            ProbeSetupState.SURFACE_VERIFICATION_CONVERGED: "Verified",
            ProbeSetupState.SURFACE_VERIFICATION_FAILED: "Failed",
            ProbeSetupState.SURFACE_VERIFICATION_CANCELLED: "Cancelled",
            ProbeSetupState.SURFACE_VERIFICATION_RECOVERY_REQUIRED: (
                "Recovery required"
            ),
        }
        label = labels.get(
            state.surface_verification_state,
            "Surface verification",
        )
        detail = state.detail.strip()
        self.surface_distance_test_status_label.setText(
            f"{label}: {detail}" if detail else label
        )

    def _submit_probe_setup(self, intent):
        if not hasattr(self.ui, "execute_probe_setup"):
            return self.show_setup_unavailable("Probe authoring")
        return self.ui.execute_probe_setup(intent)

    def _geometry_intent(self, operation):
        intent = ProbeSetupIntent()
        intent.operation = operation
        intent.approach_mode = (
            self.reference_approach_mode_dropdown.currentData()
        )
        intent.target_surface_distance_m = self._distance_value(
            self.reference_target_distance_field,
            "Target surface distance",
        )
        intent.aligned_preapproach_distance_m = self._distance_value(
            self.reference_preapproach_distance_field,
            "Aligned pre-approach distance",
        )
        return intent

    def _request_geometry_update(self):
        try:
            intent = self._geometry_intent(
                ProbeSetupIntent.OPERATION_UPDATE_GEOMETRY
            )
        except ValueError as exception:
            self._set_target_status("Unavailable", str(exception))
            return False
        return self._submit_probe_setup(intent)

    def _clear_reference_previews(self, message):
        self._reference_slot_view_ids = ["", "", ""]
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

    def _handle_reference_image_point_cleared(self):
        self.reference_pixel_value_label.setText("—")
        self.clear_reference_pixel_button.setEnabled(False)
        self._clear_selected_surface_point()

    def _handle_approach_mode_changed(self, _index=None):
        if self._probe_setup_state is None:
            return
        self._request_geometry_update()

    def _handle_target_distance_changed(self):
        presentation = self._refinement_presentation
        if (
            presentation is not None
            and presentation.recovery_required
        ):
            self.reference_target_distance_field.setText(
                f"{presentation.target_surface_distance_m:.3f}"
            )
            self.reference_preapproach_distance_field.setText(
                f"{presentation.aligned_preapproach_distance_m:.3f}"
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
        if self._probe_setup_state is None:
            return
        self._request_geometry_update()

    def _handle_dialog_distances_changed(self):
        presentation = self._refinement_presentation
        old_target = (
            presentation.target_surface_distance_m
            if presentation is not None
            else self._distance_value(
                self.reference_target_distance_field,
                "Target surface distance",
            )
        )
        old_aligned = (
            presentation.aligned_preapproach_distance_m
            if presentation is not None
            else self._distance_value(
                self.reference_preapproach_distance_field,
                "Aligned pre-approach distance",
            )
        )
        try:
            if presentation is not None and presentation.recovery_required:
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
        self._clear_live_surface_orientation()
        self._calculated_surface_probe_orientation = None
        self._calculated_surface_hand_orientation = None
        self._calculated_probe_setup = None
        self._probe_setup = None
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
        mounting = self._configured_hand_to_probe_pose()
        hand_target = probe_pose_to_hand_pose(target, mounting)
        roll, pitch, yaw = quaternion_to_rpy(hand_target.orientation)
        self.reference_target_roll_value_label.setText(
            self._format_readout_value(math.degrees(roll), 1)
        )
        self.reference_target_pitch_value_label.setText(
            self._format_readout_value(math.degrees(pitch), 1)
        )
        self.reference_target_yaw_value_label.setText(
            self._format_readout_value(
                self._normalize_degrees(math.degrees(yaw)),
                1,
            )
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
            "Object-frame hand quaternion: "
            f"x={hand_target.orientation.x:.5f}, "
            f"y={hand_target.orientation.y:.5f}, "
            f"z={hand_target.orientation.z:.5f}, "
            f"w={hand_target.orientation.w:.5f}. "
            "Object-frame probe quaternion: "
            f"x={target.orientation.x:.5f}, "
            f"y={target.orientation.y:.5f}, "
            f"z={target.orientation.z:.5f}, "
            f"w={target.orientation.w:.5f}. "
            "The probe local +X axis points toward the surface."
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
        """Start or resume one server-owned supervised refinement workflow."""
        if self._refinement_presentation is not None:
            return self.resume_refinement_dialog()
        intent = ProbeSetupIntent()
        intent.operation = ProbeSetupIntent.OPERATION_BEGIN_REFINEMENT
        return self._submit_probe_setup(intent) is not None

    def pause_refinement_dialog(self):
        """Hide the popup without discarding authoritative refinement state."""
        presentation = self._refinement_presentation
        if presentation is None:
            return True
        if presentation.pending_motion is not None:
            self.refinement_recovery_status_label.setText(
                "Wait for the active movement and settle check."
            )
            return False
        if presentation.recovery_required:
            self.refinement_recovery_status_label.setText(
                "Retract Without Saving is required before hiding this workflow."
            )
            return False
        self.inspection_workspace_splitter.setEnabled(True)
        self.start_probe_refinement_button.setText(
            "Resume Probe Point Position Refinement Workflow"
        )
        self.refinement_summary_status_label.setText(
            "Refinement paused. Reopen to continue from the current stage."
        )
        return True

    def resume_refinement_dialog(self):
        """Reopen the popup at the retained authoritative stage."""
        presentation = self._refinement_presentation
        if presentation is None:
            return False
        self.inspection_workspace_splitter.setEnabled(False)
        self.start_probe_refinement_button.setText(
            "Start Probe Point Position Refinement Workflow"
        )
        self.refinement_dialog.open_for_stage(presentation.active_stage)
        self._refresh_refinement_dialog()
        return True

    def request_close_refinement_workflow(self):
        """Treat popup Close as pause, not workflow destruction."""
        return self.pause_refinement_dialog()

    def _finish_refinement_workflow_close(self):
        self._distance_failure_requires_retraction = False
        self._retraction_failed = False
        if hasattr(self, "inspection_workspace_splitter"):
            self.inspection_workspace_splitter.setEnabled(True)
        self._refinement_presentation = None
        self.start_probe_refinement_button.setText(
            "Start Probe Point Position Refinement Workflow"
        )
        self.refinement_summary_status_label.setText(
            "Refinement workflow closed. Persisted data was preserved."
        )
        self._update_probe_setup_status_widgets()

    def handle_refinement_back(self):
        """Navigate backward without commanding movement."""
        presentation = self._require_refinement_presentation()
        if presentation.recovery_required:
            self.refinement_recovery_status_label.setText(
                "Retract Without Saving before navigating backward."
            )
            return False
        index = ProbeRefinementDialog.STAGES.index(presentation.active_stage)
        if index == 0:
            return False
        self.refinement_dialog.show_stage(
            ProbeRefinementDialog.STAGES[index - 1]
        )
        return True

    def handle_refinement_next(self):
        """Advance only when the current pose has been approved."""
        presentation = self._require_refinement_presentation()
        stage = presentation.active_stage
        if not presentation.stage_is_approved(stage):
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
        presentation = self._refinement_presentation
        if presentation is None:
            return
        presentation.active_stage = stage
        self.refinement_recovery_status_label.setText("")

    def _refresh_refinement_dialog(self):
        presentation = self._refinement_presentation
        if presentation is None or not hasattr(self, "refinement_dialog"):
            return
        for stage in RefinementStage:
            labels = self.refinement_dialog.pose_comparison_labels[stage]
            calculated = presentation.calculated_pose(stage)
            candidate = presentation.candidate_pose(stage)
            approved = presentation.approved_pose(stage)
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
            presentation.motion_states[RefinementStage.SAFE_APPROACH].value
        )
        self.alignment_step_status_label.setText(
            presentation.motion_states[RefinementStage.ALIGNMENT].value
        )
        probe_state = presentation.motion_states[RefinementStage.PROBE].value
        if presentation.surface_distance_verified:
            probe_state = "Surface Distance Verified"
        self.probe_step_status_label.setText(probe_state)

        pending = presentation.pending_motion is not None
        current = presentation.active_stage
        recovery_only = self._retraction_failed
        safe_page = current == RefinementStage.SAFE_APPROACH
        alignment_page = current == RefinementStage.ALIGNMENT
        probe_page = current == RefinementStage.PROBE
        safe_reached = (
            presentation.motion_states[RefinementStage.SAFE_APPROACH]
            == RefinementMotionState.REACHED
        )
        safe_adjustable = presentation.motion_states[
            RefinementStage.SAFE_APPROACH
        ] in (
            RefinementMotionState.REACHED,
            RefinementMotionState.FAILED,
        )
        alignment_reached = (
            presentation.motion_states[RefinementStage.ALIGNMENT]
            == RefinementMotionState.REACHED
        )
        alignment_adjustable = presentation.motion_states[
            RefinementStage.ALIGNMENT
        ] in (
            RefinementMotionState.REACHED,
            RefinementMotionState.FAILED,
        )

        safe_enabled = safe_page and not pending and not recovery_only
        self.move_calculated_approach_button.setEnabled(safe_enabled)
        self.use_current_approach_button.setEnabled(safe_enabled)
        for button in self.refinement_buttons["approach"].values():
            button.setEnabled(safe_enabled and safe_adjustable)

        alignment_enabled = (
            alignment_page
            and safe_reached
            and not pending
            and not recovery_only
        )
        self.move_aligned_pose_button.setEnabled(alignment_enabled)
        self.calculate_hand_surface_orientation_button.setEnabled(
            alignment_enabled
        )
        surface_selected = (
            self.alignment_orientation_mode_dropdown.currentData()
            == ProbeSetupMotionIntent.ALIGNMENT_ORIENTATION_CALCULATED_SURFACE
        )
        self.orient_to_calculated_surface_button.setEnabled(
            alignment_enabled
            and surface_selected
            and self._calculated_surface_probe_orientation is not None
        )
        self.use_current_alignment_button.setEnabled(
            alignment_enabled and alignment_adjustable
        )
        for button in self.refinement_buttons["alignment"].values():
            button.setEnabled(
                alignment_enabled and alignment_adjustable
            )

        test_enabled = (
            probe_page
            and alignment_reached
            and not pending
            and not self._distance_failure_requires_retraction
            and not recovery_only
        )
        self.test_surface_distance_button.setEnabled(test_enabled)
        self.retract_without_saving_button.setEnabled(
            presentation.recovery_required and not pending
        )
        distances_editable = not pending and not presentation.recovery_required
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
        self.alignment_orientation_mode_dropdown.setEnabled(
            alignment_page and not pending and not recovery_only
        )

        self.refinement_dialog.back_button.setEnabled(
            current != RefinementStage.SAFE_APPROACH
            and not pending
            and not presentation.recovery_required
        )
        self.refinement_dialog.next_button.setVisible(not probe_page)
        approved = presentation.approved_pose(current)
        self.refinement_dialog.next_button.setEnabled(
            presentation.stage_is_approved(current) and not pending
        )
        self.refinement_dialog.next_button.setText(
            "Keep Existing and Continue"
            if (
                approved is not None
                and self._poses_equivalent(
                    approved,
                    presentation.candidate_pose(current),
                )
            )
            else "Next"
        )
        self.refinement_dialog.close_button.setEnabled(not pending)
        if presentation.recovery_required:
            self.refinement_recovery_status_label.setText(
                presentation.recovery_message
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

    def _require_refinement_presentation(self):
        presentation = self._refinement_presentation
        if presentation is None:
            raise RuntimeError("Probe refinement workflow is not active")
        return presentation

    def handle_move_to_approach_pose(self):
        presentation = self._require_refinement_presentation()
        return self._send_refinement_motion(
            RefinementStage.SAFE_APPROACH,
            "safe approach candidate",
            presentation.candidate_pose(RefinementStage.SAFE_APPROACH),
        )

    def handle_move_to_aligned_pose(self):
        self._require_refinement_presentation()
        return self._send_alignment_motion(orientation_only=False)

    def handle_orient_to_calculated_surface(self):
        self._require_refinement_presentation()
        if self._calculated_surface_probe_orientation is None:
            self._show_setup_error(
                "Orient to Calculated Surface",
                ValueError("Calculate the live hand-facing surface first"),
            )
            return False
        index = self.alignment_orientation_mode_dropdown.findData(
            ProbeSetupMotionIntent.ALIGNMENT_ORIENTATION_CALCULATED_SURFACE
        )
        self.alignment_orientation_mode_dropdown.setCurrentIndex(index)
        return self._send_alignment_motion(orientation_only=True)

    def handle_calculate_hand_surface_orientation(self):
        client = getattr(self.ui, "probe_setup_client", None)
        if client is None:
            return self.show_setup_unavailable(
                "Live hand surface orientation"
            )
        future = client.calculate_surface_orientation()
        if future is None:
            return False
        self.calculated_surface_orientation_value_label.setText(
            "Calculating from live hand depth..."
        )
        return True

    def apply_live_surface_orientation(self, response):
        probe = response.probe_orientation_object
        hand = response.hand_orientation_object
        self._calculated_surface_probe_orientation = QuaternionData(
            x=float(probe.x),
            y=float(probe.y),
            z=float(probe.z),
            w=float(probe.w),
        )
        self._calculated_surface_probe_orientation.validate()
        self._calculated_surface_hand_orientation = QuaternionData(
            x=float(hand.x),
            y=float(hand.y),
            z=float(hand.z),
            w=float(hand.w),
        )
        self._calculated_surface_hand_orientation.validate()
        roll, pitch, yaw = quaternion_to_rpy(
            self._calculated_surface_hand_orientation
        )
        self.calculated_surface_orientation_value_label.setText(
            "rpy=("
            f"{math.degrees(roll):.1f}, "
            f"{math.degrees(pitch):.1f}, "
            f"{self._normalize_degrees(math.degrees(yaw)):.1f}) deg"
        )
        normal = response.surface_normal_object
        self.calculated_surface_orientation_value_label.setToolTip(
            "Object-frame surface normal: "
            f"({normal.x:.4f}, {normal.y:.4f}, {normal.z:.4f}); "
            f"samples={int(response.sample_count)}; "
            f"RMSE={float(response.plane_rmse_m):.4f} m"
        )
        self._refresh_refinement_dialog()
        return True

    def apply_live_surface_orientation_error(self, detail):
        self._clear_live_surface_orientation()
        self.calculated_surface_orientation_value_label.setText(
            f"Unavailable: {detail}"
        )
        self._refresh_refinement_dialog()
        return False

    def _clear_live_surface_orientation(self):
        self._calculated_surface_probe_orientation = None
        self._calculated_surface_hand_orientation = None
        if hasattr(self, "calculated_surface_orientation_value_label"):
            self.calculated_surface_orientation_value_label.setText(
                "Not calculated"
            )
            self.calculated_surface_orientation_value_label.setToolTip("")

    def _handle_alignment_orientation_mode_changed(self, _index=None):
        self._refresh_refinement_dialog()

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
            presentation = self._require_refinement_presentation()
            stage_value = {
                "approach": RefinementStage.SAFE_APPROACH,
                "alignment": RefinementStage.ALIGNMENT,
            }.get(stage)
            if stage_value is None:
                raise ValueError(
                    "Probe geometry is refined only at the aligned "
                    "pre-approach pose"
                )
            if presentation.active_stage != stage_value:
                raise ValueError("The requested refinement stage is inactive")
            if presentation.motion_states[stage_value] not in (
                RefinementMotionState.REACHED,
                RefinementMotionState.FAILED,
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
        if not hasattr(
            self.ui,
            "execute_probe_surface_verification",
        ):
            return self.show_setup_unavailable("Surface verification")
        request_id = self.ui.execute_probe_surface_verification()
        if request_id is None:
            self.surface_distance_test_status_label.setText(
                "Surface verification action is unavailable"
            )
            return False
        self.surface_distance_test_status_label.setText(
            "Surface verification running"
        )
        return True

    @staticmethod
    def _refinement_delta(action, translation_step, rotation_step):
        translations = {
            "up": Vector3Data(x=0.0, y=0.0, z=translation_step),
            "down": Vector3Data(x=0.0, y=0.0, z=-translation_step),
            "left": Vector3Data(x=0.0, y=translation_step, z=0.0),
            "right": Vector3Data(x=0.0, y=-translation_step, z=0.0),
            "front": Vector3Data(x=translation_step, y=0.0, z=0.0),
            "back": Vector3Data(x=-translation_step, y=0.0, z=0.0),
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


    def handle_use_current_as_probe(self):
        self._show_setup_error(
            "Capture Probe Pose",
            ValueError(
                "Probe approval requires verified live surface distance "
                "and Approve and Retract"
            ),
        )
        return False


    def _send_alignment_motion(self, orientation_only):
        intent = ProbeSetupMotionIntent()
        intent.operation = (
            ProbeSetupMotionIntent.OPERATION_MOVE_ALIGNED_PREAPPROACH
        )
        intent.frame = ProbeSetupMotionIntent.FRAME_SENSOR
        intent.alignment_orientation_mode = int(
            self.alignment_orientation_mode_dropdown.currentData()
        )
        intent.orientation_only = bool(orientation_only)
        if (
            intent.alignment_orientation_mode
            == ProbeSetupMotionIntent.ALIGNMENT_ORIENTATION_CALCULATED_SURFACE
        ):
            orientation = self._calculated_surface_probe_orientation
            if orientation is None:
                self._show_setup_error(
                    "Move to Aligned Pose",
                    ValueError(
                        "Calculate the live hand-facing surface first"
                    ),
                )
                return False
            intent.has_calculated_surface_orientation = True
            intent.calculated_surface_orientation_object.x = orientation.x
            intent.calculated_surface_orientation_object.y = orientation.y
            intent.calculated_surface_orientation_object.z = orientation.z
            intent.calculated_surface_orientation_object.w = orientation.w
        self._write_motion_tolerances(intent)
        label = (
            "calculated surface orientation"
            if orientation_only
            else "aligned pre-approach candidate"
        )
        return self._submit_probe_motion(intent, label)

    def _send_refinement_motion(
        self,
        stage,
        label,
        probe_pose_object,
        updates_candidate=False,
        axial_correction_m=0.0,
    ):
        if stage == RefinementStage.SAFE_APPROACH:
            operation = (
                ProbeSetupMotionIntent.OPERATION_MOVE_SAFE_APPROACH
            )
        elif stage == RefinementStage.ALIGNMENT:
            operation = (
                ProbeSetupMotionIntent.OPERATION_MOVE_ALIGNED_PREAPPROACH
            )
        else:
            return self.show_setup_unavailable(
                "Controlled probe-axis motion"
            )
        intent = ProbeSetupMotionIntent()
        intent.operation = operation
        intent.frame = ProbeSetupMotionIntent.FRAME_SENSOR
        self._write_motion_tolerances(intent)
        return self._submit_probe_motion(intent, label)

    def _send_refinement_relative_motion(
        self,
        stage,
        label,
        translation,
        pitch_rad,
        yaw_rad,
    ):
        operations = {
            RefinementStage.SAFE_APPROACH: (
                ProbeSetupMotionIntent.OPERATION_ADJUST_SAFE_APPROACH
            ),
            RefinementStage.ALIGNMENT: (
                ProbeSetupMotionIntent.OPERATION_ADJUST_ALIGNED_PREAPPROACH
            ),
        }
        if stage not in operations:
            return self.show_setup_unavailable(
                "Controlled probe-axis adjustment"
            )
        intent = ProbeSetupMotionIntent()
        intent.operation = operations[stage]
        intent.frame = self._selected_refinement_frame_code()
        intent.translation.x = translation.x
        intent.translation.y = translation.y
        intent.translation.z = translation.z
        intent.pitch_rad = pitch_rad
        intent.yaw_rad = yaw_rad
        self._write_motion_tolerances(intent)
        return self._submit_probe_motion(intent, label)

    def _selected_refinement_frame_code(self):
        selection = self.refine_frame_dropdown.currentData()
        values = {
            REFINEMENT_FRAME_SENSOR: ProbeSetupMotionIntent.FRAME_SENSOR,
            REFINEMENT_FRAME_HAND: ProbeSetupMotionIntent.FRAME_HAND,
            REFINEMENT_FRAME_TAG: ProbeSetupMotionIntent.FRAME_TAG,
            REFINEMENT_FRAME_BODY: ProbeSetupMotionIntent.FRAME_BODY,
            REFINEMENT_FRAME_MAP: ProbeSetupMotionIntent.FRAME_MAP,
        }
        try:
            return values[selection]
        except KeyError as exception:
            raise ValueError("Select a valid refinement frame") from exception

    def _write_motion_tolerances(self, intent):
        intent.position_tolerance_m = self._distance_value(
            self.probe_position_tolerance_field,
            "Position tolerance",
        )
        intent.orientation_tolerance_rad = self._distance_value(
            self.probe_orientation_tolerance_field,
            "Orientation tolerance",
        )

    def _submit_probe_motion(self, intent, label):
        client = getattr(self.ui, "probe_setup_client", None)
        if client is None:
            return self.show_setup_unavailable("Probe setup motion")
        request_id = client.execute_motion(intent)
        if request_id is None:
            return False
        self._set_status_text(f"Probe setup motion submitted: {label}")
        return True

    def handle_refinement_emergency_stop(self):
        """Request the application-level emergency stop."""
        self.ui.handle_emergency_stop()

    def _configured_hand_to_probe_pose(self):
        sensor_id = self._selected_sensor_id
        if not sensor_id:
            sensor_id = self.sensor_id_field.currentData()
        definition = self._sensor_definitions.get(sensor_id)
        if definition is None:
            return PoseData.identity()
        return deepcopy(definition.hand_to_probe)

    def _active_probe_frame(self):
        if self._selected_sensor_id:
            return sensor_probe_frame(self._selected_sensor_id)

        sensor_id = self.sensor_id_field.currentData()
        if not sensor_id:
            raise ValueError("Sensor mounting must be selected")
        return sensor_probe_frame(sensor_id)


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

    def refresh_setup_state(self):
        """Request the current immutable probe setup snapshot."""
        client = getattr(self.ui, "probe_setup_client", None)
        if client is None or not client.context_id:
            return False
        intent = ProbeSetupIntent()
        intent.operation = ProbeSetupIntent.OPERATION_REFRESH
        return self._submit_probe_setup(intent) is not None

    def refresh_saved_definitions(
        self,
        desired_object_id=None,
        desired_routine_id=None,
    ):
        """Refresh definitions through the server-owned setup context."""
        return self.refresh_setup_state()

    def apply_setup_state(self, state):
        """Render one authoritative probe setup snapshot."""
        if not isinstance(state, ProbeSetupState):
            raise TypeError("Expected a ProbeSetupState message")
        view = probe_setup_state_to_view(state)
        previous_views = tuple(self._reference_slot_view_ids)
        self._probe_setup_state = state
        self._selected_sensor_id = state.selected_sensor_id
        self._render_surface_verification_state(state)
        self._apply_object_and_routine_lists(state)
        self._apply_probe_setup_view(view)
        signature = (
            state.selected_object_id,
            state.selected_routine_id,
            tuple(state.reference_view_ids),
        )
        if signature != getattr(self, "_preview_signature", None):
            self._preview_signature = signature
            self._request_reference_previews(state)
        elif previous_views != tuple(self._reference_slot_view_ids):
            self._restore_authoritative_selection()
        if state.state == ProbeSetupState.STATE_FAILED:
            self._set_setup_status("Unavailable", state.detail)
        else:
            self._set_status_text(state.detail)
        return True

    def _apply_object_and_routine_lists(self, state):
        self.saved_object_dropdown.blockSignals(True)
        self.saved_object_dropdown.clear()
        self.saved_object_dropdown.addItem("Select saved object", None)
        for object_id in state.object_ids:
            self.saved_object_dropdown.addItem(object_id, object_id)
        object_index = self.saved_object_dropdown.findData(
            state.selected_object_id
        )
        self.saved_object_dropdown.setCurrentIndex(
            object_index if object_index >= 0 else 0
        )
        self.saved_object_dropdown.blockSignals(False)

        self.routine_parent_object_dropdown.blockSignals(True)
        self.routine_parent_object_dropdown.clear()
        self.routine_parent_object_dropdown.addItem(
            "Select existing object",
            None,
        )
        for object_id in state.object_ids:
            self.routine_parent_object_dropdown.addItem(
                object_id,
                object_id,
            )
        parent_index = self.routine_parent_object_dropdown.findData(
            state.selected_object_id
        )
        self.routine_parent_object_dropdown.setCurrentIndex(
            parent_index if parent_index >= 0 else 0
        )
        self.routine_parent_object_dropdown.blockSignals(False)

        self.saved_routine_dropdown.blockSignals(True)
        self.saved_routine_dropdown.clear()
        self.saved_routine_dropdown.addItem(
            "Select saved routine",
            None,
        )
        for routine_id in state.routine_ids:
            self.saved_routine_dropdown.addItem(routine_id, routine_id)
        routine_index = self.saved_routine_dropdown.findData(
            state.selected_routine_id
        )
        self.saved_routine_dropdown.setCurrentIndex(
            routine_index if routine_index >= 0 else 0
        )
        self.saved_routine_dropdown.blockSignals(False)

        self.delete_object_button.setEnabled(
            bool(state.selected_object_id)
        )
        self.delete_routine_button.setEnabled(
            bool(state.selected_object_id and state.selected_routine_id)
        )
        if state.selected_sensor_id:
            self._pending_sensor_selection = state.selected_sensor_id
            self._populate_sensor_dropdown(state.selected_sensor_id)

    def _apply_probe_setup_view(self, view):
        state = view.message
        self._selected_surface_point = view.projected_point
        self._selected_surface_normal = view.surface_normal
        self._surface_normal_error = state.surface_normal_error
        self._selected_approach_direction = view.approach_direction
        self._selected_surface_target = view.surface_target
        self._calculated_probe_setup = view.calculated_setup
        self._probe_setup = view.setup
        self._synchronize_refinement_presentation(view.refinement)

        if not state.has_reference_pixel:
            self._clear_selected_surface_point()
            return
        self.reference_pixel_value_label.setText(
            f"u={state.reference_pixel_u}, v={state.reference_pixel_v}"
        )
        self.clear_reference_pixel_button.setEnabled(True)

        if view.projected_point is None:
            self._set_projection_unavailable(
                "Unavailable",
                state.validation_error or state.detail,
            )
            return
        point = view.projected_point
        self.reference_surface_frame_value_label.setText(point.frame_id)
        self.reference_surface_frame_value_label.setToolTip(point.frame_id)
        self.reference_surface_x_value_label.setText(
            self._format_readout_value(point.point_camera.x, 3)
        )
        self.reference_surface_y_value_label.setText(
            self._format_readout_value(point.point_camera.y, 3)
        )
        self.reference_surface_z_value_label.setText(
            self._format_readout_value(point.point_camera.z, 3)
        )
        mapped = point.mapped_pixel
        sampled = point.sampled_pixel
        self.reference_depth_pixel_value_label.setText(
            f"{sampled.u},{sampled.v}"
            if sampled == mapped
            else f"{mapped.u},{mapped.v} to {sampled.u},{sampled.v}"
        )
        self._set_projection_status("Ready")

        if view.surface_normal is None:
            self.reference_normal_x_value_label.setText("N/A")
            self.reference_normal_y_value_label.setText("N/A")
            self.reference_normal_z_value_label.setText("N/A")
            self.reference_normal_samples_value_label.setText("N/A")
            self.reference_normal_rmse_value_label.setText("N/A")
            self._set_normal_status(
                "Unavailable",
                state.surface_normal_error,
            )
        else:
            normal = view.surface_normal
            self.reference_normal_x_value_label.setText(
                self._format_readout_value(normal.normal_camera.x, 3)
            )
            self.reference_normal_y_value_label.setText(
                self._format_readout_value(normal.normal_camera.y, 3)
            )
            self.reference_normal_z_value_label.setText(
                self._format_readout_value(normal.normal_camera.z, 3)
            )
            self.reference_normal_samples_value_label.setText(
                str(normal.sample_count)
            )
            self.reference_normal_rmse_value_label.setText(
                self._format_readout_value(normal.plane_rmse_m, 4)
            )
            self._set_normal_status("Ready")

        if view.approach_direction is None:
            self._set_approach_status(
                "Unavailable",
                state.validation_error or state.detail,
            )
        else:
            source_text = {
                APPROACH_SOURCE_SURFACE_FIT: "Surface fit",
                APPROACH_SOURCE_TAG_X_SELECTED: "Calibrated tag +X",
            }.get(view.approach_direction.source, "Unknown")
            self.reference_approach_source_value_label.setText(source_text)
            self._set_approach_status("Ready")

        if view.setup is None:
            self._clear_selected_surface_target()
            return
        self.reference_target_distance_field.setText(
            f"{state.target_surface_distance_m:.3f}"
        )
        self.reference_preapproach_distance_field.setText(
            f"{state.aligned_preapproach_distance_m:.3f}"
        )
        self._display_probe_setup("Authoritative")

    def _synchronize_refinement_presentation(self, presentation):
        previous = self._refinement_presentation
        if presentation is None:
            if previous is not None:
                self._finish_refinement_workflow_close()
            return
        if previous is not None:
            presentation.active_stage = previous.active_stage
        self._refinement_presentation = presentation
        self._distance_failure_requires_retraction = (
            presentation.recovery_required
        )
        self._retraction_failed = False
        if previous is None:
            self._clear_live_surface_orientation()
            self.resume_refinement_dialog()
        elif self.refinement_dialog.isVisible():
            self.inspection_workspace_splitter.setEnabled(False)
        self._refresh_refinement_dialog()

    def _request_reference_previews(self, state):
        self._clear_reference_previews("Loading reference preview")
        self._reference_slot_view_ids = ["", "", ""]
        if not state.selected_routine_id:
            self.reference_view_status_label.setText(
                "Reference view: no routine selected"
            )
            self._set_reference_camera_defaults()
            return
        if not state.reference_view_ids:
            self.reference_view_status_label.setText(
                "Reference view: not captured"
            )
            self._set_reference_camera_defaults()
            return
        client = getattr(self.ui, "probe_setup_client", None)
        if client is None:
            self.reference_view_status_label.setText(
                "Reference view: remote preview unavailable"
            )
            return
        for view_id in state.reference_view_ids:
            client.request_preview(view_id)

    def apply_reference_preview(self, response):
        """Render one preview only if it belongs to the current snapshot."""
        state = self._probe_setup_state
        if (
            state is None
            or response.reference_view_id not in state.reference_view_ids
        ):
            return False
        slot_index = int(response.slot_index)
        if slot_index < 0 or slot_index >= len(self.reference_view_widgets):
            self.apply_reference_preview_error(
                response.reference_view_id,
                f"Invalid reference preview slot: {slot_index}",
            )
            return False
        region = ImageRegion(
            x=int(response.selectable_x),
            y=int(response.selectable_y),
            width=int(response.selectable_width),
            height=int(response.selectable_height),
        )
        widget = self.reference_view_widgets[slot_index]
        widget.blockSignals(True)
        widget.set_ros_image(response.image, valid_region=region)
        widget.blockSignals(False)
        self._reference_slot_view_ids[slot_index] = (
            response.reference_view_id
        )
        dropdown = self.reference_camera_dropdowns[slot_index]
        dropdown.blockSignals(True)
        camera_index = dropdown.findData(response.camera_id)
        dropdown.setCurrentIndex(camera_index if camera_index >= 0 else 0)
        dropdown.blockSignals(False)
        self.reference_view_status_label.setText(
            "Reference view: remote preview ready"
        )
        self._restore_authoritative_selection()
        return True

    def apply_reference_preview_error(self, view_id, detail):
        state = self._probe_setup_state
        if state is None or view_id not in state.reference_view_ids:
            return False
        self.reference_view_status_label.setText(
            "Reference view: preview unavailable"
        )
        self.reference_view_status_label.setToolTip(detail)
        return True

    def _restore_authoritative_selection(self):
        state = self._probe_setup_state
        if state is None or not state.has_reference_pixel:
            return
        try:
            slot_index = self._reference_slot_view_ids.index(
                state.selected_reference_view_id
            )
        except ValueError:
            return
        widget = self.reference_view_widgets[slot_index]
        widget.blockSignals(True)
        widget.set_selected_image_point(
            ImagePoint(
                u=int(state.reference_pixel_u),
                v=int(state.reference_pixel_v),
            )
        )
        widget.blockSignals(False)
        self._active_reference_slot = slot_index

    def _load_selected_object(self, _index=None):
        object_id = self.saved_object_dropdown.currentData()
        state = self._probe_setup_state
        if not object_id:
            return False
        if state is not None and object_id == state.selected_object_id:
            return True
        intent = ProbeSetupIntent()
        intent.operation = ProbeSetupIntent.OPERATION_SELECT_OBJECT
        intent.object_id = object_id
        return self._submit_probe_setup(intent) is not None

    def _load_selected_routine(self, _index=None):
        object_id = self.saved_object_dropdown.currentData()
        routine_id = self.saved_routine_dropdown.currentData()
        state = self._probe_setup_state
        if not object_id or not routine_id:
            return False
        if (
            state is not None
            and object_id == state.selected_object_id
            and routine_id == state.selected_routine_id
        ):
            return True
        intent = ProbeSetupIntent()
        intent.operation = ProbeSetupIntent.OPERATION_SELECT_ROUTINE
        intent.object_id = object_id
        intent.routine_id = routine_id
        self._clear_live_surface_orientation()
        return self._submit_probe_setup(intent) is not None

    def _refresh_routine_parent_objects(
        self,
        desired_object_id="",
        object_ids=None,
    ):
        state = self._probe_setup_state
        values = list(object_ids or (
            state.object_ids if state is not None else ()
        ))
        self.routine_parent_object_dropdown.blockSignals(True)
        self.routine_parent_object_dropdown.clear()
        self.routine_parent_object_dropdown.addItem(
            "Select existing object",
            None,
        )
        for object_id in values:
            self.routine_parent_object_dropdown.addItem(
                object_id,
                object_id,
            )
        index = self.routine_parent_object_dropdown.findData(
            desired_object_id
        )
        self.routine_parent_object_dropdown.setCurrentIndex(
            index if index >= 0 else 0
        )
        self.routine_parent_object_dropdown.blockSignals(False)

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
        for routine_id in routines:
            self.saved_routine_dropdown.addItem(routine_id, routine_id)
        index = self.saved_routine_dropdown.findData(desired_routine_id)
        self.saved_routine_dropdown.setCurrentIndex(
            index if index >= 0 else 0
        )
        self.saved_routine_dropdown.blockSignals(False)

    def _schedule_repository_refresh(self):
        for delay_ms in (250, 1000, 3500):
            QTimer.singleShot(delay_ms, self.refresh_setup_state)

    def handle_create_object(self):
        """Submit one server-owned object creation transaction."""
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
        intent = ProbeSetupIntent()
        intent.operation = ProbeSetupIntent.OPERATION_CREATE_OBJECT
        intent.object_id = object_id
        intent.object_display_name = display_name
        intent.reference_tag_id = tag_id
        intent.reference_tag_family = tag_family
        submitted = self._submit_probe_setup(intent) is not None
        if submitted:
            self.object_id_field.clear()
            self.object_display_name_field.clear()
            self.reference_tag_id_field.clear()
            self.reference_tag_family_field.setText("36h11")
        return submitted

    def handle_create_routine(self):
        """Submit one server-owned routine creation transaction."""
        object_id = self.routine_parent_object_dropdown.currentData()
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
        if None in (routine_id, display_name) or not sensor_id:
            return False
        intent = ProbeSetupIntent()
        intent.operation = ProbeSetupIntent.OPERATION_CREATE_ROUTINE
        intent.object_id = object_id
        intent.routine_id = routine_id
        intent.routine_display_name = display_name
        intent.sensor_id = sensor_id
        submitted = self._submit_probe_setup(intent) is not None
        if submitted:
            self.routine_id_field.clear()
            self.routine_display_name_field.clear()
        return submitted

    def handle_capture_reference_view(self):
        """Keep capture unavailable until its execution action is migrated."""
        return self.show_setup_unavailable("Reference capture")

    def handle_delete_object(self):
        """Submit one server-owned object deletion transaction."""
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
        intent = ProbeSetupIntent()
        intent.operation = ProbeSetupIntent.OPERATION_DELETE_OBJECT
        intent.object_id = object_id
        return self._submit_probe_setup(intent) is not None

    def handle_delete_routine(self):
        """Submit one server-owned routine deletion transaction."""
        object_id = self.saved_object_dropdown.currentData()
        routine_id = self.saved_routine_dropdown.currentData()
        if not object_id or not routine_id:
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
        intent = ProbeSetupIntent()
        intent.operation = ProbeSetupIntent.OPERATION_DELETE_ROUTINE
        intent.object_id = object_id
        intent.routine_id = routine_id
        return self._submit_probe_setup(intent) is not None

    def handle_use_current_as_approach(self):
        """Request approval of the server-observed safe pose."""
        intent = ProbeSetupIntent()
        intent.operation = ProbeSetupIntent.OPERATION_APPROVE_SAFE_POSE
        return self._submit_probe_setup(intent) is not None

    def handle_use_current_alignment(self):
        """Request approval of the server-observed aligned pose."""
        intent = ProbeSetupIntent()
        intent.operation = ProbeSetupIntent.OPERATION_APPROVE_ALIGNED_POSE
        return self._submit_probe_setup(intent) is not None

    def handle_approve_and_retract(self):
        """Fail closed until approval, save, and retraction are coordinated."""
        return self.show_setup_unavailable(
            "Probe approval, persistence, and retraction"
        )

    def handle_retract_without_saving(self):
        """Fail closed until retraction is delegated to the server."""
        return self.show_setup_unavailable("Probe retraction")
