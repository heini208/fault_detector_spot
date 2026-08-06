"""Inspection setup controls."""

import json
import math
from copy import deepcopy

from bosdyn.client.frame_helpers import HAND_FRAME_NAME
from PyQt5.QtCore import QLocale, Qt, QTimer
from PyQt5.QtGui import QDoubleValidator, QIntValidator
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

from fault_detector_msgs.msg import ComplexCommand, TagElement
from rclpy.duration import Duration
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
import tf2_ros

from fault_detector_spot.inspection.models import (
    ImagePoint,
    InspectionObject,
    InspectionRoutine,
    PoseData,
    QuaternionData,
    ReferenceTag,
    Vector3Data,
)
from fault_detector_spot.inspection.multi_reference_view_repository import (
    CapturedReferenceView,
    MultiReferenceViewRepository,
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
    relative_pose,
)
from fault_detector_spot.inspection.reference_view_surface_target import (
    quaternion_to_rpy,
    resolve_reference_surface_target,
)

from ..commands.command_ids import CommandID, OrientationModes
from .collapsible_section import CollapsibleSection
from .UIControlHelper import UIControlHelper
from .reference_view_widget import ReferenceViewWidget


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
        self._probe_setup = None
        self._tf_buffer = None
        self._tf_listener = None
        self.management_dialog = None
        super().__init__(parent_ui)
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
        self.management_dialog.resize(620, 430)

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

        self.sensor_id_field = QLineEdit("bmm150")
        self.sensor_id_field.setPlaceholderText("Sensor ID")
        routine_layout.addRow("Sensor ID:", self.sensor_id_field)

        self.probe_frame_field = QLineEdit("sensor_tip")
        self.probe_frame_field.setPlaceholderText("Probe frame")
        routine_layout.addRow("Probe frame:", self.probe_frame_field)

        routine_buttons = QHBoxLayout()
        self.create_routine_button = QPushButton("Create Routine")
        self.create_routine_button.clicked.connect(
            self.handle_create_routine
        )
        routine_buttons.addWidget(self.create_routine_button)
        routine_buttons.addStretch()
        routine_layout.addRow(routine_buttons)
        dialog_layout.addWidget(routine_group)

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
        self.management_status_label.setText(
            "Create a new object, or select an existing object "
            "for the new routine."
        )
        self.management_dialog.show()
        self.management_dialog.raise_()
        self.management_dialog.activateWindow()

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
        self.reference_preapproach_distance_field = QLineEdit("0.15")
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
            "Move to Approach Pose"
        )
        self.use_current_approach_button = QPushButton(
            "Use Current as Approach"
        )
        self.move_aligned_pose_button = QPushButton(
            "Move to Aligned Pose"
        )
        self.use_current_alignment_button = QPushButton(
            "Use Current Alignment"
        )
        self.move_probe_pose_button = QPushButton("Move to Probe Pose")
        self.use_current_probe_button = QPushButton(
            "Use Current as Probe"
        )

        self.approach_step_status_label = QLabel("Waiting for target")
        self.alignment_step_status_label = QLabel("Waiting for target")
        self.probe_step_status_label = QLabel("Waiting for alignment")
        self.save_approach_status_label = QLabel("Not approved")
        self.save_alignment_status_label = QLabel("Not approved")
        self.save_probe_status_label = QLabel("Not approved")

        self.probe_point_id_field = QLineEdit()
        self.probe_point_id_field.setPlaceholderText("Probe point ID")
        self.probe_point_display_name_field = QLineEdit()
        self.probe_point_display_name_field.setPlaceholderText(
            "Display name"
        )
        self.probe_position_tolerance_field = QLineEdit("0.01")
        self.probe_orientation_tolerance_field = QLineEdit("0.087")
        self.probe_measurement_duration_field = QLineEdit("1.0")
        self.save_probe_point_button = QPushButton("Save Probe Point")
        self.save_probe_point_button.setEnabled(False)
        self.save_probe_point_status_label = QLabel(
            "Saving will be enabled by the persistence patch."
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
        self.move_probe_pose_button.clicked.connect(
            self.handle_move_to_probe_pose
        )
        self.use_current_probe_button.clicked.connect(
            self.handle_use_current_as_probe
        )

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
        target_layout.addWidget(QLabel("Target distance [m]:"), 0, 0)
        target_layout.addWidget(
            self.reference_target_distance_field,
            0,
            1,
        )
        target_layout.addWidget(QLabel("Aligned distance [m]:"), 0, 2)
        target_layout.addWidget(
            self.reference_preapproach_distance_field,
            0,
            3,
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

        target_layout.addWidget(QLabel("Aligned position [m]:"), 4, 0)
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

        layout.addWidget(
            self._make_refine_stage_group(
                "1. Obstacle-safe approach",
                self.approach_step_status_label,
                self.move_calculated_approach_button,
                self.use_current_approach_button,
                "Move to the calculated pose, adjust the sensor manually, "
                "then approve the current pose.",
            )
        )
        layout.addWidget(
            self._make_refine_stage_group(
                "2. Surface alignment",
                self.alignment_step_status_label,
                self.move_aligned_pose_button,
                self.use_current_alignment_button,
                "Move directly in front of the surface, fine-tune position "
                "and orientation, then approve the alignment.",
            )
        )
        layout.addWidget(
            self._make_refine_stage_group(
                "3. Probe pose",
                self.probe_step_status_label,
                self.move_probe_pose_button,
                self.use_current_probe_button,
                "Move to the final target, make small adjustments, then "
                "approve the final probe pose.",
            )
        )
        layout.addStretch()
        return widget

    @staticmethod
    def _make_refine_stage_group(
        title,
        status_label,
        move_button,
        approve_button,
        description,
    ):
        group = QGroupBox(title)
        layout = QVBoxLayout(group)
        description_label = QLabel(description)
        description_label.setWordWrap(True)
        layout.addWidget(description_label)

        status_row = QHBoxLayout()
        status_row.addWidget(QLabel("Status:"))
        status_row.addWidget(status_label)
        status_row.addStretch()
        layout.addLayout(status_row)

        buttons = QHBoxLayout()
        buttons.addWidget(move_button)
        buttons.addWidget(approve_button)
        buttons.addStretch()
        layout.addLayout(buttons)
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
            "Surface alignment:",
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
        layout.addWidget(self.save_probe_point_button)
        layout.addStretch()
        return widget

    def _set_probe_setup_buttons_enabled(self, enabled):
        setup_buttons = (
            self.move_calculated_approach_button,
            self.use_current_approach_button,
            self.move_aligned_pose_button,
            self.use_current_alignment_button,
        )
        for button in setup_buttons:
            button.setEnabled(enabled)
        probe_enabled = (
            enabled
            and self._probe_setup is not None
            and self._probe_setup.surface_alignment_approved
        )
        self.move_probe_pose_button.setEnabled(probe_enabled)
        self.use_current_probe_button.setEnabled(probe_enabled)

    def _update_probe_setup_status_widgets(self):
        setup = self._probe_setup
        if setup is None:
            self.approach_step_status_label.setText("Waiting for target")
            self.alignment_step_status_label.setText("Waiting for target")
            self.probe_step_status_label.setText("Waiting for alignment")
            self.save_approach_status_label.setText("Not approved")
            self.save_alignment_status_label.setText("Not approved")
            self.save_probe_status_label.setText("Not approved")
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

    @staticmethod
    def _distance_validator(parent):
        validator = QDoubleValidator(0.001, 10.0, 3, parent)
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
        if self._selected_approach_direction is None:
            self._clear_selected_surface_target()
            return
        self._resolve_selected_surface_target()

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
        self._clear_selected_surface_target()
        if (
            self._selected_approach_direction is None
            or self._reference_view is None
        ):
            return
        try:
            target_distance = self._distance_value(
                self.reference_target_distance_field,
                "Target surface distance",
            )
            preapproach_distance = self._distance_value(
                self.reference_preapproach_distance_field,
                "Aligned pre-approach distance",
            )
            result = resolve_reference_surface_target(
                approach_direction=self._selected_approach_direction,
                controlled_frame_pose_object=(
                    self._reference_view.controlled_frame_pose_object
                ),
                target_surface_distance_m=target_distance,
                aligned_preapproach_distance_m=preapproach_distance,
            )
        except ValueError as exception:
            self._set_target_status("Unavailable", str(exception))
            return

        self._selected_surface_target = result
        self._probe_setup = initialize_reference_probe_setup(result)
        self._set_probe_setup_buttons_enabled(True)
        self._display_probe_setup("Calculated")

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
            f"alignment approved={setup.surface_alignment_approved}; "
            f"probe approved={setup.probe_pose_approved}. "
            "Nothing is persisted until a later Save Probe Point step."
        )

    def handle_move_to_approach_pose(self):
        self._move_setup_pose("safe_approach_pose_object", "approach pose")

    def handle_move_to_aligned_pose(self):
        self._move_setup_pose(
            "aligned_preapproach_pose_object",
            "aligned pre-approach pose",
        )

    def handle_move_to_probe_pose(self):
        self._move_setup_pose("probe_pose_object", "probe pose")

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
            current = self._current_probe_pose_object()
            self._probe_setup = approve_safe_approach_pose(
                self._require_probe_setup(),
                current,
            )
        except Exception as exception:
            self._show_setup_error("Capture Approach Pose", exception)
            return
        self._display_probe_setup("Approach approved")
        self._set_status_text("Current probe pose approved as approach pose")

    def handle_use_current_alignment(self):
        try:
            current = self._current_probe_pose_object()
            self._probe_setup = approve_surface_alignment_pose(
                self._require_probe_setup(),
                current,
            )
        except Exception as exception:
            self._show_setup_error("Capture Surface Alignment", exception)
            return
        self._display_probe_setup("Alignment approved")
        self._set_status_text("Current probe pose approved as alignment")

    def handle_use_current_as_probe(self):
        try:
            current = self._current_probe_pose_object()
            self._probe_setup = approve_probe_pose(
                self._require_probe_setup(),
                current,
            )
        except Exception as exception:
            self._show_setup_error("Capture Probe Pose", exception)
            return
        self._display_probe_setup("Probe approved")
        self._set_status_text(
            "Current probe pose approved as final probe pose"
        )

    def _move_transient_probe_pose(self, probe_pose_object, label):
        try:
            command = self._build_probe_pose_command(probe_pose_object)
        except Exception as exception:
            self._show_setup_error("Move Probe Setup", exception)
            return
        self.complex_command_publisher.publish(command)
        self._set_status_text(f"Command sent: move to {label}")

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
            if routine is not None and routine.probe_frame.strip():
                return routine.probe_frame.strip()

        probe_frame = self.probe_frame_field.text().strip()
        if not probe_frame:
            raise ValueError("Probe frame must not be empty")
        return probe_frame

    def _lookup_pose(self, target_frame, source_frame):
        if self._tf_buffer is None:
            raise RuntimeError("TF is unavailable in the inspection UI")
        transform = self._tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            Time(),
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
        visible_tags = getattr(self.ui, "visible_tags", {})
        tag = visible_tags.get(tag_id)
        if tag is None:
            raise ValueError(
                f"Reference tag {tag_id} must be currently visible"
            )
        if not tag.pose.header.frame_id.strip():
            raise ValueError("Reference tag pose frame is empty")
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

    def _required_text(self, field, label):
        value = field.text().strip()
        if value:
            return value
        self.show_warning("Missing Input", f"Enter {label}.")
        return None

    def _new_command(self, command_id):
        command = ComplexCommand()
        command.command = self.ui.build_basic_command(command_id)
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
        self.create_routine_button.setEnabled(bool(object_ids))

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
            self.sensor_id_field.setText(routine.sensor_id)
            self.probe_frame_field.setText(routine.probe_frame)

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
            self.reference_view_status_label.setText(
                "Reference view: captured datasets unavailable"
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
        sensor_id = self._required_text(
            self.sensor_id_field,
            "a sensor ID",
        )
        probe_frame = self._required_text(
            self.probe_frame_field,
            "a probe frame",
        )
        if None in (
            routine_id,
            display_name,
            sensor_id,
            probe_frame,
        ):
            return False

        routine = InspectionRoutine(
            routine_id=routine_id,
            display_name=display_name,
            sensor_id=sensor_id,
            probe_frame=probe_frame,
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
        command.inspection.routine.probe_frame = probe_frame
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

