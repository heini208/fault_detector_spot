"""Presentation-only sensor mount workspace."""

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QComboBox,
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
    QSpacerItem,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)


class SensorControls(QWidget):
    """Render the sensor mount feature without owning application state."""

    def __init__(self, parent=None):
        """Build the presentation-only sensor mount workspace."""
        super().__init__(parent)
        self._build_ui()

    def _build_ui(self) -> None:
        page_layout = QVBoxLayout(self)
        page_layout.setContentsMargins(0, 0, 0, 0)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)

        content = QWidget()
        content_layout = QVBoxLayout(content)
        content_layout.setContentsMargins(12, 10, 12, 12)
        content_layout.setSpacing(10)

        content_layout.addWidget(self._build_header())
        content_layout.addWidget(self._build_attachment_group())

        body = QHBoxLayout()
        body.setSpacing(10)
        body.addWidget(self._build_registry_group(), 5)
        body.addWidget(self._build_definition_group(), 6)
        content_layout.addLayout(body)

        content_layout.addWidget(self._build_configuration_group())
        content_layout.addStretch()

        scroll.setWidget(content)
        page_layout.addWidget(scroll)

    def _build_header(self) -> QWidget:
        header = QWidget()
        layout = QVBoxLayout(header)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(2)

        title = QLabel("Sensor Mounts")
        title.setStyleSheet("font-size: 18px; font-weight: bold;")

        subtitle = QLabel(
            "Layout prototype for registered hand-mounted sensor "
            "configurations and the currently attached mount."
        )
        subtitle.setWordWrap(True)

        prototype = QLabel(
            "Presentation only — no controls on this page are connected."
        )
        prototype.setStyleSheet("color: palette(mid);")
        prototype.setWordWrap(True)

        layout.addWidget(title)
        layout.addWidget(subtitle)
        layout.addWidget(prototype)
        return header

    def _build_attachment_group(self) -> QGroupBox:
        group = QGroupBox("Current attachment")
        layout = QGridLayout(group)
        layout.setHorizontalSpacing(12)
        layout.setVerticalSpacing(6)

        self.attachment_name_value = QLabel("—")
        self.attachment_status_value = QLabel("Not connected")
        self.attachment_probe_frame_value = QLabel("—")
        self.attachment_revision_value = QLabel("—")

        layout.addWidget(QLabel("Selected mount:"), 0, 0)
        layout.addWidget(self.attachment_name_value, 0, 1)
        layout.addWidget(QLabel("Status:"), 0, 2)
        layout.addWidget(self.attachment_status_value, 0, 3)
        layout.addWidget(QLabel("Probe frame:"), 1, 0)
        layout.addWidget(self.attachment_probe_frame_value, 1, 1)
        layout.addWidget(QLabel("Attachment revision:"), 1, 2)
        layout.addWidget(self.attachment_revision_value, 1, 3)

        self.active_mount_dropdown = QComboBox()
        self.active_mount_dropdown.addItem("Select a registered mount")
        self.active_mount_dropdown.setEnabled(False)

        self.select_mount_button = QPushButton("Select Mount")
        self.confirm_attachment_button = QPushButton("Confirm Attachment")
        self.clear_attachment_button = QPushButton("No Sensor Attached")

        for button in (
            self.select_mount_button,
            self.confirm_attachment_button,
            self.clear_attachment_button,
        ):
            button.setEnabled(False)

        action_row = QHBoxLayout()
        action_row.addWidget(self.active_mount_dropdown, 1)
        action_row.addWidget(self.select_mount_button)
        action_row.addWidget(self.confirm_attachment_button)
        action_row.addWidget(self.clear_attachment_button)

        layout.addLayout(action_row, 2, 0, 1, 4)
        layout.setColumnStretch(1, 1)
        layout.setColumnStretch(3, 1)
        return group

    def _build_registry_group(self) -> QGroupBox:
        group = QGroupBox("Registered sensor mounts")
        layout = QVBoxLayout(group)

        self.mount_table = QTableWidget(0, 4)
        self.mount_table.setHorizontalHeaderLabels(
            (
                "Display name",
                "Mount ID",
                "Probe frame",
                "Calibration",
            )
        )
        self.mount_table.horizontalHeader().setStretchLastSection(True)
        self.mount_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.mount_table.setSelectionMode(QTableWidget.SingleSelection)
        self.mount_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.mount_table.setMinimumHeight(180)

        self.empty_registry_label = QLabel(
            "Registered mounts will appear here once the backend is "
            "connected."
        )
        self.empty_registry_label.setAlignment(Qt.AlignCenter)
        self.empty_registry_label.setStyleSheet("color: palette(mid);")

        self.new_mount_button = QPushButton("New Sensor Mount")
        self.retire_mount_button = QPushButton("Retire Selected")
        self.new_mount_button.setEnabled(False)
        self.retire_mount_button.setEnabled(False)

        actions = QHBoxLayout()
        actions.addWidget(self.new_mount_button)
        actions.addWidget(self.retire_mount_button)
        actions.addStretch()

        layout.addWidget(self.mount_table)
        layout.addWidget(self.empty_registry_label)
        layout.addLayout(actions)
        return group

    def _build_definition_group(self) -> QGroupBox:
        group = QGroupBox("Sensor mount definition")
        outer = QVBoxLayout(group)

        form = QFormLayout()
        form.setFieldGrowthPolicy(QFormLayout.ExpandingFieldsGrow)

        self.mount_id_field = QLineEdit()
        self.mount_id_field.setPlaceholderText("e.g. bmm150_mount")
        self.display_name_field = QLineEdit()
        self.display_name_field.setPlaceholderText(
            "Human-readable sensor mount name"
        )
        self.probe_frame_field = QLineEdit()
        self.probe_frame_field.setPlaceholderText(
            "Derived motion / probe frame"
        )
        self.probe_frame_field.setReadOnly(True)

        form.addRow("Mount ID:", self.mount_id_field)
        form.addRow("Display name:", self.display_name_field)
        form.addRow("Probe frame:", self.probe_frame_field)

        translation = self._vector_fields(
            ("X", "Y", "Z"),
            "m",
        )
        rotation = self._vector_fields(
            ("Roll", "Pitch", "Yaw"),
            "deg",
        )
        self.translation_fields = translation[0]
        self.rotation_fields = rotation[0]

        form.addRow("Hand → probe translation:", translation[1])
        form.addRow("Hand → probe rotation:", rotation[1])

        self.calibration_status_value = QLabel("Not saved")
        form.addRow("Calibration status:", self.calibration_status_value)

        self.save_mount_button = QPushButton("Save Sensor Mount")
        self.discard_mount_button = QPushButton("Discard Changes")
        self.save_mount_button.setEnabled(False)
        self.discard_mount_button.setEnabled(False)

        actions = QHBoxLayout()
        actions.addStretch()
        actions.addWidget(self.discard_mount_button)
        actions.addWidget(self.save_mount_button)

        outer.addLayout(form)
        outer.addItem(
            QSpacerItem(
                0,
                6,
                QSizePolicy.Minimum,
                QSizePolicy.Fixed,
            )
        )
        outer.addLayout(actions)
        return group

    def _build_configuration_group(self) -> QGroupBox:
        group = QGroupBox("Configuration and calibration")
        layout = QGridLayout(group)
        layout.setHorizontalSpacing(16)
        layout.setVerticalSpacing(5)

        manual_title = QLabel("Manual calibration")
        manual_title.setStyleSheet("font-weight: bold;")
        manual_text = QLabel(
            "Review or enter the calibrated hand-to-probe transform "
            "in the sensor mount definition above."
        )
        manual_text.setWordWrap(True)

        automatic_title = QLabel("Automatic configuration")
        automatic_title.setStyleSheet("font-weight: bold;")
        automatic_text = QLabel(
            "Reserved for a later workflow that can determine the probe "
            "tip and, if useful, the physical mount size."
        )
        automatic_text.setWordWrap(True)

        sensing_title = QLabel("Sensing elements")
        sensing_title.setStyleSheet("font-weight: bold;")
        sensing_text = QLabel(
            "A mount may contain multiple measurement sensors. Their "
            "individual geometry can be added later without changing the "
            "single motion probe frame."
        )
        sensing_text.setWordWrap(True)

        self.start_configuration_button = QPushButton(
            "Start Automatic Configuration"
        )
        self.manage_sensing_elements_button = QPushButton(
            "Manage Sensing Elements"
        )
        self.start_configuration_button.setEnabled(False)
        self.manage_sensing_elements_button.setEnabled(False)

        layout.addWidget(manual_title, 0, 0)
        layout.addWidget(manual_text, 1, 0)
        layout.addWidget(automatic_title, 0, 1)
        layout.addWidget(automatic_text, 1, 1)
        layout.addWidget(self.start_configuration_button, 2, 1)
        layout.addWidget(sensing_title, 0, 2)
        layout.addWidget(sensing_text, 1, 2)
        layout.addWidget(self.manage_sensing_elements_button, 2, 2)

        for column in range(3):
            layout.setColumnStretch(column, 1)
        return group

    @staticmethod
    def _vector_fields(labels, unit):
        widget = QWidget()
        layout = QHBoxLayout(widget)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(5)

        fields = []
        for label_text in labels:
            label = QLabel(f"{label_text}:")
            field = QLineEdit()
            field.setPlaceholderText("0.0")
            field.setMaximumWidth(85)
            fields.append(field)
            layout.addWidget(label)
            layout.addWidget(field)

        layout.addWidget(QLabel(unit))
        layout.addStretch()
        return tuple(fields), widget

    def add_preview_mount(
        self,
        display_name: str,
        mount_id: str,
        probe_frame: str,
        calibration: str,
    ) -> None:
        """Add one presentation-only row for layout evaluation."""
        row = self.mount_table.rowCount()
        self.mount_table.insertRow(row)
        for column, value in enumerate(
            (display_name, mount_id, probe_frame, calibration)
        ):
            self.mount_table.setItem(
                row,
                column,
                QTableWidgetItem(value),
            )
        self.empty_registry_label.setVisible(False)
