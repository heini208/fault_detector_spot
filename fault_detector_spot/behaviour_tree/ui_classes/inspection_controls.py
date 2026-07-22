"""Inspection setup controls."""

from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QIntValidator
from PyQt5.QtWidgets import (
    QCheckBox,
    QComboBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
)

from fault_detector_msgs.msg import ComplexCommand

from fault_detector_spot.inspection.models import ImagePoint
from fault_detector_spot.inspection.object_repository import (
    ObjectRepository,
)
from fault_detector_spot.inspection.reference_view_depth_projection import (
    project_reference_pixel,
)

from ..commands.command_ids import CommandID
from .UIControlHelper import UIControlHelper
from .reference_view_widget import ReferenceViewWidget


class InspectionControls(UIControlHelper):
    """Build and publish inspection-definition setup commands."""

    def __init__(self, parent_ui):
        """Create controls backed by the configured object repository."""
        self.object_repository = ObjectRepository(
            getattr(parent_ui, "inspection_object_root", None)
        )
        self._selected_definition = None
        self._reference_rgb_size = None
        self._reference_depth_image = None
        self._reference_camera_info = None
        self._selected_surface_point = None
        super().__init__(parent_ui)
        self.refresh_saved_definitions()

    def add_rows(self, layout):
        """Add the rows constructed during initialization."""
        for row in self.rows:
            layout.addLayout(row)

    def init_ros_communication(self):
        """Use the complex-command publisher owned by the main UI."""
        self.complex_command_publisher = self.ui.complex_command_publisher

    def make_rows(self):
        """Create the inspection setup rows."""
        return [
            self._make_saved_definitions_row(),
            self._make_storage_row(),
            self._make_object_identity_row(),
            self._make_reference_tag_row(),
            self._make_routine_identity_row(),
            self._make_sensor_row(),
            self._make_reference_view_row(),
            self._make_reference_view_preview_row(),
        ]

    def _make_saved_definitions_row(self):
        row = QHBoxLayout()
        row.addWidget(QLabel("Saved Definitions:"))

        self.saved_object_dropdown = QComboBox()
        self.saved_object_dropdown.currentIndexChanged.connect(
            self._load_selected_object
        )
        row.addWidget(self.saved_object_dropdown)

        self.saved_routine_dropdown = QComboBox()
        self.saved_routine_dropdown.currentIndexChanged.connect(
            self._load_selected_routine
        )
        row.addWidget(self.saved_routine_dropdown)

        self.refresh_definitions_button = QPushButton("Refresh")
        self.refresh_definitions_button.clicked.connect(
            self.refresh_saved_definitions
        )
        row.addWidget(self.refresh_definitions_button)
        return row

    def _make_storage_row(self):
        row = QHBoxLayout()
        self.storage_path_label = QLabel(
            f"Storage: {self.object_repository.root_dir}"
        )
        row.addWidget(self.storage_path_label)
        self.reference_view_status_label = QLabel(
            "Reference view: no routine selected"
        )
        row.addWidget(self.reference_view_status_label)
        row.addStretch()
        return row

    def _make_object_identity_row(self):
        row = QHBoxLayout()
        row.addWidget(QLabel("Inspection Object:"))

        self.object_id_field = QLineEdit()
        self.object_id_field.setPlaceholderText("Object ID")
        row.addWidget(self.object_id_field)

        self.object_display_name_field = QLineEdit()
        self.object_display_name_field.setPlaceholderText("Display name")
        row.addWidget(self.object_display_name_field)
        return row

    def _make_reference_tag_row(self):
        row = QHBoxLayout()
        row.addWidget(QLabel("Reference Tag:"))

        self.reference_tag_id_field = QLineEdit()
        self.reference_tag_id_field.setPlaceholderText("Tag ID")
        self.reference_tag_id_field.setValidator(
            QIntValidator(0, 2147483647, self.reference_tag_id_field)
        )
        row.addWidget(self.reference_tag_id_field)

        self.reference_tag_family_field = QLineEdit("36h11")
        self.reference_tag_family_field.setPlaceholderText("Tag family")
        row.addWidget(self.reference_tag_family_field)

        self.create_object_button = QPushButton("Create Object")
        self.create_object_button.clicked.connect(self.handle_create_object)
        row.addWidget(self.create_object_button)

        self.delete_object_button = QPushButton("Delete Object")
        self.delete_object_button.clicked.connect(self.handle_delete_object)
        row.addWidget(self.delete_object_button)
        return row

    def _make_routine_identity_row(self):
        row = QHBoxLayout()
        row.addWidget(QLabel("Inspection Routine:"))

        self.routine_id_field = QLineEdit()
        self.routine_id_field.setPlaceholderText("Routine ID")
        row.addWidget(self.routine_id_field)

        self.routine_display_name_field = QLineEdit()
        self.routine_display_name_field.setPlaceholderText("Display name")
        row.addWidget(self.routine_display_name_field)
        return row

    def _make_sensor_row(self):
        row = QHBoxLayout()
        row.addWidget(QLabel("Sensor:"))

        self.sensor_id_field = QLineEdit("bmm150")
        self.sensor_id_field.setPlaceholderText("Sensor ID")
        row.addWidget(self.sensor_id_field)

        self.probe_frame_field = QLineEdit("sensor_tip")
        self.probe_frame_field.setPlaceholderText("Probe frame")
        row.addWidget(self.probe_frame_field)

        self.create_routine_button = QPushButton("Create Routine")
        self.create_routine_button.clicked.connect(
            self.handle_create_routine
        )
        row.addWidget(self.create_routine_button)

        self.delete_routine_button = QPushButton("Delete Routine")
        self.delete_routine_button.clicked.connect(
            self.handle_delete_routine
        )
        row.addWidget(self.delete_routine_button)
        return row

    def _make_reference_view_row(self):
        row = QHBoxLayout()
        row.addWidget(QLabel("Reference View:"))

        self.replace_reference_view_checkbox = QCheckBox(
            "Replace existing"
        )
        row.addWidget(self.replace_reference_view_checkbox)

        self.capture_reference_view_button = QPushButton(
            "Capture Reference View"
        )
        self.capture_reference_view_button.clicked.connect(
            self.handle_capture_reference_view
        )
        row.addWidget(self.capture_reference_view_button)
        row.addStretch()
        return row

    def _make_reference_view_preview_row(self):
        row = QHBoxLayout()
        row.addWidget(QLabel("Reference Image:"))
        self.reference_view_widget = ReferenceViewWidget()
        self.reference_pixel_label = QLabel("Selected pixel: none")
        self.reference_surface_point_label = QLabel(
            "Surface point: none"
        )
        self.clear_reference_pixel_button = QPushButton("Clear Point")
        self.clear_reference_pixel_button.setEnabled(False)
        self.clear_reference_pixel_button.clicked.connect(
            self.reference_view_widget.clear_selection
        )
        self.reference_view_widget.image_point_changed.connect(
            self._handle_reference_image_point_changed
        )
        self.reference_view_widget.image_point_cleared.connect(
            self._handle_reference_image_point_cleared
        )
        row.addWidget(self.reference_view_widget, 1)
        row.addWidget(self.reference_pixel_label)
        row.addWidget(self.reference_surface_point_label)
        row.addWidget(self.clear_reference_pixel_button)
        return row

    def _handle_reference_image_point_changed(self, u, v):
        self.reference_pixel_label.setText(
            f"Selected pixel: u={u}, v={v}"
        )
        self.clear_reference_pixel_button.setEnabled(True)
        self._project_selected_reference_pixel(u, v)

    def _handle_reference_image_point_cleared(self):
        self.reference_pixel_label.setText("Selected pixel: none")
        self.clear_reference_pixel_button.setEnabled(False)
        self._clear_selected_surface_point()

    @property
    def selected_surface_point(self):
        """Return the transient projected reference surface point."""
        return self._selected_surface_point

    def _clear_selected_surface_point(self):
        self._selected_surface_point = None
        self.reference_surface_point_label.setText(
            "Surface point: none"
        )

    def _project_selected_reference_pixel(self, u, v):
        self._selected_surface_point = None
        if (
            self._reference_rgb_size is None
            or self._reference_depth_image is None
            or self._reference_camera_info is None
        ):
            self.reference_surface_point_label.setText(
                "Surface point: reference depth unavailable"
            )
            return
        depth_size = (
            self._reference_depth_image.width,
            self._reference_depth_image.height,
        )
        if self._reference_rgb_size != depth_size:
            self.reference_surface_point_label.setText(
                "Surface point unavailable: reference RGB and "
                "registered depth dimensions do not match"
            )
            return

        try:
            result = project_reference_pixel(
                ImagePoint(u=u, v=v),
                self._reference_depth_image,
                self._reference_camera_info,
            )
        except ValueError as exception:
            self.reference_surface_point_label.setText(
                f"Surface point unavailable: {exception}"
            )
            return

        self._selected_surface_point = result
        point = result.point_camera
        sample_suffix = ""
        if result.sampled_pixel != result.requested_pixel:
            sample_suffix = (
                f"; depth pixel u={result.sampled_pixel.u}, "
                f"v={result.sampled_pixel.v}"
            )
        self.reference_surface_point_label.setText(
            f"Surface point [{result.frame_id}]: "
            f"x={point.x:.3f}, y={point.y:.3f}, "
            f"z={point.z:.3f} m{sample_suffix}"
        )

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

    def refresh_saved_definitions(self):
        """Reload selectable objects and routines from persistent storage."""
        desired_object_id = ""
        if hasattr(self, "object_id_field"):
            desired_object_id = self.object_id_field.text().strip()
        if not desired_object_id:
            desired_object_id = (
                self.saved_object_dropdown.currentData() or ""
            )

        self.saved_object_dropdown.blockSignals(True)
        self.saved_object_dropdown.clear()
        self.saved_object_dropdown.addItem(
            "Select saved object",
            None,
        )
        for object_id in self.object_repository.list_object_ids():
            self.saved_object_dropdown.addItem(object_id, object_id)

        selected_index = self.saved_object_dropdown.findData(
            desired_object_id
        )
        self.saved_object_dropdown.setCurrentIndex(
            selected_index if selected_index >= 0 else 0
        )
        self.saved_object_dropdown.blockSignals(False)
        self._load_selected_object()

    def _load_selected_object(self, _index=None):
        object_id = self.saved_object_dropdown.currentData()
        if not object_id:
            self._selected_definition = None
            self._populate_routine_dropdown([])
            self.reference_view_status_label.setText(
                "Reference view: no routine selected"
            )
            self.reference_view_widget.clear_preview(
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
            self.reference_view_widget.clear_preview(
                "Reference view unavailable"
            )
            return

        desired_routine_id = self.routine_id_field.text().strip()
        if not desired_routine_id:
            desired_routine_id = (
                self.saved_routine_dropdown.currentData() or ""
            )

        self._selected_definition = definition
        self.object_id_field.setText(definition.object_id)
        self.object_display_name_field.setText(definition.display_name)
        self.reference_tag_id_field.setText(
            str(definition.reference_tag.tag_id)
        )
        self.reference_tag_family_field.setText(
            definition.reference_tag.tag_family
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
        self._reference_rgb_size = None
        self._reference_depth_image = None
        self._reference_camera_info = None
        self._clear_selected_surface_point()
        routine_id = self.saved_routine_dropdown.currentData()
        if not routine_id or self._selected_definition is None:
            self.reference_view_status_label.setText(
                "Reference view: no routine selected"
            )
            self.reference_view_widget.clear_preview(
                "No reference view selected"
            )
            return

        routine = self._selected_definition.get_routine(routine_id)
        if routine is None:
            self.reference_view_status_label.setText(
                "Reference view: routine no longer exists"
            )
            self.reference_view_widget.clear_preview(
                "Reference view unavailable"
            )
            return

        self.routine_id_field.setText(routine.routine_id)
        self.routine_display_name_field.setText(routine.display_name)
        self.sensor_id_field.setText(routine.sensor_id)
        self.probe_frame_field.setText(routine.probe_frame)
        if routine.reference_view is None:
            status = "Reference view: not captured"
            self.reference_view_status_label.setText(status)
            self.reference_view_widget.clear_preview(
                "No reference view captured"
            )
            return

        status = (
            "Reference view: captured, "
            f"{routine.reference_view.reference_dataset_path}"
        )
        try:
            rgb_image, depth_image, camera_info = (
                self.object_repository.load_reference_dataset(
                    self._selected_definition.object_id,
                    routine.routine_id,
                )
            )
            self._reference_rgb_size = (rgb_image.width, rgb_image.height)
            self._reference_depth_image = depth_image
            self._reference_camera_info = camera_info
            self.reference_view_widget.set_ros_image(rgb_image)
        except Exception as exception:
            self.reference_view_status_label.setText(
                f"{status}; preview unavailable: {exception}"
            )
            self.reference_view_widget.clear_preview(
                "Reference view unavailable"
            )
            return

        self.reference_view_status_label.setText(status)

    def _schedule_repository_refresh(self):
        if self.node is None:
            return
        for delay_ms in (250, 1000, 3500):
            QTimer.singleShot(delay_ms, self.refresh_saved_definitions)

    def handle_create_object(self):
        """Publish an explicit inspection-object creation command."""
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
        except ValueError:
            self.show_warning(
                "Invalid Input",
                "Reference tag ID must be a non-negative integer.",
            )
            return False
        if tag_id < 0:
            self.show_warning(
                "Invalid Input",
                "Reference tag ID must be a non-negative integer.",
            )
            return False

        command = self._new_command(CommandID.CREATE_INSPECTION_OBJECT)
        command.inspection.object.object_id = object_id
        command.inspection.object.display_name = display_name
        command.inspection.object.reference_tag_id = tag_id
        command.inspection.object.reference_tag_family = tag_family
        self._publish(command)
        return True

    def handle_create_routine(self):
        """Publish an explicit inspection-routine creation command."""
        object_id = self._required_text(
            self.object_id_field,
            "an object ID",
        )
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
            object_id,
            routine_id,
            display_name,
            sensor_id,
            probe_frame,
        ):
            return False

        command = self._new_command(CommandID.CREATE_INSPECTION_ROUTINE)
        command.inspection.object.object_id = object_id
        command.inspection.routine.routine_id = routine_id
        command.inspection.routine.display_name = display_name
        command.inspection.routine.sensor_id = sensor_id
        command.inspection.routine.probe_frame = probe_frame
        self._publish(command)
        return True

    def handle_capture_reference_view(self):
        """Publish a reference-view capture command."""
        object_id = self._required_text(
            self.object_id_field,
            "an object ID",
        )
        routine_id = self._required_text(
            self.routine_id_field,
            "a routine ID",
        )
        if None in (object_id, routine_id):
            return False

        replace_existing = self.replace_reference_view_checkbox.isChecked()
        if replace_existing and not self.ask_question(
            "Replace Reference View",
            (
                f"Replace the saved reference view for "
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
        self._publish(command)
        return True

    def handle_delete_object(self):
        """Publish deletion of the selected saved object."""
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

        command = self._new_command(
            CommandID.DELETE_INSPECTION_OBJECT
        )
        command.inspection.object.object_id = object_id
        self._publish(command)
        return True

    def handle_delete_routine(self):
        """Publish deletion of the selected saved routine."""
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

        command = self._new_command(
            CommandID.DELETE_INSPECTION_ROUTINE
        )
        command.inspection.object.object_id = object_id
        command.inspection.routine.routine_id = routine_id
        self._publish(command)
        return True
