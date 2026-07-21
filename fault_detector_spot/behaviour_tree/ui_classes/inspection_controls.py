"""Inspection setup controls."""

from PyQt5.QtGui import QIntValidator
from PyQt5.QtWidgets import (
    QCheckBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
)

from fault_detector_msgs.msg import ComplexCommand

from ..commands.command_ids import CommandID
from .UIControlHelper import UIControlHelper


class InspectionControls(UIControlHelper):
    """Build and publish inspection-definition setup commands."""

    def init_ros_communication(self):
        """Use the complex-command publisher owned by the main UI."""
        self.complex_command_publisher = self.ui.complex_command_publisher

    def make_rows(self):
        """Create the inspection setup rows."""
        return [
            self._make_object_identity_row(),
            self._make_reference_tag_row(),
            self._make_routine_identity_row(),
            self._make_sensor_row(),
            self._make_reference_view_row(),
        ]

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
