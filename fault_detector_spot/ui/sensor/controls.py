"""Presentation-only sensor mount workspace."""

from dataclasses import dataclass

from PyQt5.QtCore import QLocale, Qt, pyqtSignal
from PyQt5.QtGui import QDoubleValidator
from PyQt5.QtWidgets import (
    QComboBox,
    QFormLayout,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHeaderView,
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


@dataclass(frozen=True)
class SensorChannelIntent:
    """Describe one channel entered in the sensor-mount form."""

    channel_id: str
    topic: str
    message_type: str
    source_kind: str = "ros_topic"


_DEFAULT_GEOMETRY_CHANNEL = SensorChannelIntent(
    channel_id="spot_geometry",
    topic="",
    message_type="",
    source_kind="spot_geometry",
)


@dataclass(frozen=True)
class SensorDefinitionIntent:
    """Describe one manually entered physical sensor transform."""

    sensor_id: str
    display_name: str
    translation_m: tuple
    rotation_degrees: tuple
    channels: tuple = ()


class SensorControls(QWidget):
    """Render sensor registry state and emit user intents."""

    select_requested = pyqtSignal(str)
    create_requested = pyqtSignal(object)
    update_requested = pyqtSignal(object)
    delete_requested = pyqtSignal(str)

    def __init__(self, parent=None):
        """Build the sensor mount workspace."""
        super().__init__(parent)
        self._definitions = {}
        self._attachment_state = None
        self._head_connection_state = None
        self._topic_message_types = {}
        self._editing_sensor_id = ""
        self._build_ui()
        self._connect_intents()
        self._set_channel_rows((_DEFAULT_GEOMETRY_CHANNEL,))

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
            "Registered hand-mounted sensor transforms and the currently "
            "confirmed physical attachment."
        )
        subtitle.setWordWrap(True)

        layout.addWidget(title)
        layout.addWidget(subtitle)
        return header

    def _build_attachment_group(self) -> QGroupBox:
        group = QGroupBox("Current attachment")
        layout = QGridLayout(group)
        layout.setHorizontalSpacing(12)
        layout.setVerticalSpacing(6)

        self.attachment_name_value = QLabel("—")
        self.attachment_status_value = QLabel("Waiting for state")
        self.attachment_probe_frame_value = QLabel("—")
        self.head_connection_value = QLabel("Waiting for state")
        self.head_connection_value.setWordWrap(True)

        layout.addWidget(QLabel("Selected mount:"), 0, 0)
        layout.addWidget(self.attachment_name_value, 0, 1)
        layout.addWidget(QLabel("Status:"), 0, 2)
        layout.addWidget(self.attachment_status_value, 0, 3)
        layout.addWidget(QLabel("Probe frame:"), 1, 0)
        layout.addWidget(self.attachment_probe_frame_value, 1, 1, 1, 3)
        layout.addWidget(QLabel("Sensor head:"), 2, 0)
        layout.addWidget(self.head_connection_value, 2, 1, 1, 3)

        self.active_mount_dropdown = QComboBox()
        self.active_mount_dropdown.addItem(
            "No registered sensor mounts",
            "",
        )
        self.active_mount_dropdown.setEnabled(False)

        self.select_mount_button = QPushButton("Select Mount")
        self.clear_attachment_button = QPushButton("Remove Sensor")

        self.select_mount_button.setEnabled(False)
        self.clear_attachment_button.setEnabled(False)

        action_row = QHBoxLayout()
        action_row.addWidget(self.active_mount_dropdown, 1)
        action_row.addWidget(self.select_mount_button)
        action_row.addWidget(self.clear_attachment_button)

        layout.addLayout(action_row, 3, 0, 1, 4)
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
                "Transform source",
            )
        )
        self.mount_table.horizontalHeader().setStretchLastSection(True)
        self.mount_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.mount_table.setSelectionMode(QTableWidget.SingleSelection)
        self.mount_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.mount_table.setMinimumHeight(180)

        self.empty_registry_label = QLabel(
            "No registered physical sensor mounts."
        )
        self.empty_registry_label.setAlignment(Qt.AlignCenter)
        self.empty_registry_label.setStyleSheet("color: palette(mid);")

        self.edit_mount_button = QPushButton("Edit Sensor")
        self.delete_mount_button = QPushButton("Delete Selected")
        self.edit_mount_button.setEnabled(False)
        self.delete_mount_button.setEnabled(False)

        actions = QHBoxLayout()
        actions.addWidget(self.edit_mount_button)
        actions.addWidget(self.delete_mount_button)
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
        self.mount_id_field.setPlaceholderText("e.g. test or bmm150_mount")
        self.detected_mount_dropdown = QComboBox()
        self.detected_mount_dropdown.addItem(
            "No connected sensor heads",
            "",
        )
        self.detected_mount_dropdown.setEnabled(False)
        self.use_detected_mount_button = QPushButton("Use ID")
        self.use_detected_mount_button.setEnabled(False)
        detected_id_row = QWidget()
        detected_id_layout = QHBoxLayout(detected_id_row)
        detected_id_layout.setContentsMargins(0, 0, 0, 0)
        detected_id_layout.setSpacing(6)
        detected_id_layout.addWidget(self.detected_mount_dropdown, 1)
        detected_id_layout.addWidget(self.use_detected_mount_button)
        self.display_name_field = QLineEdit()
        self.display_name_field.setPlaceholderText(
            "Human-readable sensor mount name"
        )
        self.probe_frame_field = QLineEdit()
        self.probe_frame_field.setPlaceholderText(
            "Derived motion / probe frame"
        )
        self.probe_frame_field.setReadOnly(True)

        form.addRow("Detected head ID:", detected_id_row)
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

        self.transform_status_value = QLabel(
            "Manual transform. Enter a new mount ID and save."
        )
        self.transform_status_value.setWordWrap(True)
        form.addRow("Transform status:", self.transform_status_value)

        self.save_mount_button = QPushButton("Save Sensor Mount")
        self.discard_mount_button = QPushButton("Discard Changes")

        actions = QHBoxLayout()
        actions.addStretch()
        actions.addWidget(self.discard_mount_button)
        actions.addWidget(self.save_mount_button)

        outer.addLayout(form)
        outer.addWidget(self._build_channels_group())
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

    def _build_channels_group(self) -> QGroupBox:
        group = QGroupBox()
        layout = QVBoxLayout(group)

        self.channel_toggle_button = QPushButton(
            "Show acquisition channels"
        )
        self.channel_toggle_button.setCheckable(True)
        self.channel_toggle_button.toggled.connect(
            self._set_channels_expanded
        )

        self.channel_content = QWidget()
        content_layout = QVBoxLayout(self.channel_content)
        content_layout.setContentsMargins(0, 0, 0, 0)

        help_text = QLabel(
            "Add ROS streams or derived Spot geometry to each measurement. "
            "Available topics and types are suggested from the live ROS "
            "graph; offline values can still be entered manually."
        )
        help_text.setWordWrap(True)

        self.channel_table = QTableWidget(0, 4)
        self.channel_table.setHorizontalHeaderLabels(
            ("Channel ID", "Source", "ROS topic", "Message type")
        )
        channel_header = self.channel_table.horizontalHeader()
        channel_header.setSectionResizeMode(
            0,
            QHeaderView.ResizeToContents,
        )
        channel_header.setSectionResizeMode(
            1,
            QHeaderView.ResizeToContents,
        )
        channel_header.setSectionResizeMode(2, QHeaderView.Stretch)
        channel_header.setSectionResizeMode(3, QHeaderView.Stretch)
        self.channel_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.channel_table.setSelectionMode(QTableWidget.SingleSelection)
        self.channel_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.channel_table.setMinimumHeight(120)

        editor = QGridLayout()
        self.channel_id_field = QLineEdit()
        self.channel_id_field.setPlaceholderText("e.g. magnetic_field")
        self.channel_source_field = QComboBox()
        self.channel_source_field.addItem("ROS topic", "ros_topic")
        self.channel_source_field.addItem(
            "Spot geometry",
            "spot_geometry",
        )
        self.channel_topic_field = QComboBox()
        self.channel_topic_field.setEditable(True)
        self.channel_topic_field.setInsertPolicy(QComboBox.NoInsert)
        self.channel_topic_field.lineEdit().setPlaceholderText(
            "/sensors/example/topic"
        )
        self.channel_message_type_field = QComboBox()
        self.channel_message_type_field.setEditable(True)
        self.channel_message_type_field.setInsertPolicy(QComboBox.NoInsert)
        self.channel_message_type_field.lineEdit().setPlaceholderText(
            "sensor_msgs/msg/MagneticField"
        )

        editor.addWidget(QLabel("Channel ID:"), 0, 0)
        editor.addWidget(self.channel_id_field, 0, 1)
        editor.addWidget(QLabel("Source:"), 1, 0)
        editor.addWidget(self.channel_source_field, 1, 1)
        editor.addWidget(QLabel("ROS topic:"), 2, 0)
        editor.addWidget(self.channel_topic_field, 2, 1)
        editor.addWidget(QLabel("Message type:"), 3, 0)
        editor.addWidget(self.channel_message_type_field, 3, 1)
        editor.setColumnStretch(1, 1)

        self.save_channel_button = QPushButton("Add Channel")
        self.new_channel_button = QPushButton("New Channel")
        self.remove_channel_button = QPushButton("Remove Selected")
        self.remove_channel_button.setEnabled(False)
        actions = QHBoxLayout()
        actions.addWidget(self.save_channel_button)
        actions.addWidget(self.new_channel_button)
        actions.addWidget(self.remove_channel_button)
        actions.addStretch()

        content_layout.addWidget(help_text)
        content_layout.addWidget(self.channel_table)
        content_layout.addLayout(editor)
        content_layout.addLayout(actions)

        layout.addWidget(self.channel_toggle_button)
        layout.addWidget(self.channel_content)
        self._set_channels_expanded(False)
        return group

    def _set_channels_expanded(self, expanded: bool) -> None:
        self.channel_content.setVisible(expanded)
        self.channel_toggle_button.setText(
            "Hide acquisition channels"
            if expanded
            else "Show acquisition channels"
        )

    def _build_configuration_group(self) -> QGroupBox:
        group = QGroupBox("Configuration")
        layout = QGridLayout(group)
        layout.setHorizontalSpacing(16)
        layout.setVerticalSpacing(5)

        manual_title = QLabel("Manual transform")
        manual_title.setStyleSheet("font-weight: bold;")
        manual_text = QLabel(
            "Enter the transform from the Spot hand frame to the sensor "
            "probe frame above. This is manual configuration, not an "
            "automatic calibration result."
        )
        manual_text.setWordWrap(True)

        automatic_title = QLabel("Automatic calibration")
        automatic_title.setStyleSheet("font-weight: bold;")
        automatic_text = QLabel(
            "Reserved for a later workflow that can determine the probe "
            "transform automatically."
        )
        automatic_text.setWordWrap(True)

        self.start_configuration_button = QPushButton(
            "Start Automatic Calibration"
        )
        self.start_configuration_button.setEnabled(False)

        layout.addWidget(manual_title, 0, 0)
        layout.addWidget(manual_text, 1, 0)
        layout.addWidget(automatic_title, 0, 1)
        layout.addWidget(automatic_text, 1, 1)
        layout.addWidget(self.start_configuration_button, 2, 1)

        layout.setColumnStretch(0, 1)
        layout.setColumnStretch(1, 1)
        return group

    def _connect_intents(self) -> None:
        self.select_mount_button.clicked.connect(
            self._request_selected_mount
        )
        self.clear_attachment_button.clicked.connect(
            self._request_no_sensor
        )
        self.edit_mount_button.clicked.connect(
            self.edit_selected_sensor
        )
        self.delete_mount_button.clicked.connect(
            self._request_sensor_deletion
        )
        self.mount_table.itemSelectionChanged.connect(
            self._refresh_registry_actions
        )
        self.save_mount_button.clicked.connect(
            self._request_sensor_save
        )
        self.discard_mount_button.clicked.connect(
            self.clear_sensor_definition
        )
        self.use_detected_mount_button.clicked.connect(
            self._use_detected_mount_id
        )
        self.mount_id_field.textChanged.connect(
            self._update_probe_frame_preview
        )
        self.channel_table.itemSelectionChanged.connect(
            self._populate_selected_channel
        )
        self.channel_topic_field.currentTextChanged.connect(
            self._apply_topic_type_suggestion
        )
        self.channel_source_field.currentIndexChanged.connect(
            self._apply_channel_source
        )
        self.save_channel_button.clicked.connect(
            self._save_channel_row
        )
        self.new_channel_button.clicked.connect(
            self._clear_channel_editor
        )
        self.remove_channel_button.clicked.connect(
            self._remove_selected_channel
        )

    def apply_definitions(self, definitions) -> None:
        """Render the authoritative physical sensor registry."""
        selected_id = self._selected_table_sensor_id()
        dropdown_id = self.active_mount_dropdown.currentData() or ""
        self._definitions = {
            definition.sensor_id: definition
            for definition in definitions
        }
        self.mount_table.setRowCount(0)
        self.active_mount_dropdown.blockSignals(True)
        self.active_mount_dropdown.clear()

        for definition in definitions:
            self._append_definition_row(definition)
            self.active_mount_dropdown.addItem(
                definition.display_name,
                definition.sensor_id,
            )

        has_definitions = bool(definitions)
        self.empty_registry_label.setVisible(not has_definitions)
        self.active_mount_dropdown.setEnabled(has_definitions)
        self.select_mount_button.setEnabled(has_definitions)
        if has_definitions:
            index = self.active_mount_dropdown.findData(dropdown_id)
            if index < 0 and self._attachment_state is not None:
                index = self.active_mount_dropdown.findData(
                    self._attachment_state.selected_sensor_id
                )
            if index < 0:
                index = 0
            self.active_mount_dropdown.setCurrentIndex(index)
        else:
            self.active_mount_dropdown.addItem(
                "No registered sensor mounts",
                "",
            )

        self.active_mount_dropdown.blockSignals(False)
        self._restore_table_selection(selected_id)
        self._render_attachment_state()
        self._refresh_registry_actions()

    def apply_attachment_state(self, state) -> None:
        """Render authoritative physical attachment state."""
        self._attachment_state = state
        self._render_attachment_state()
        self._refresh_registry_actions()
        self._refresh_save_availability()

    def apply_topic_suggestions(self, suggestions) -> None:
        """Offer live graph topics and types without restricting input."""
        self._topic_message_types = {
            suggestion.topic: tuple(suggestion.message_types)
            for suggestion in suggestions
        }
        current_topic = self.channel_topic_field.currentText()
        self.channel_topic_field.blockSignals(True)
        self.channel_topic_field.clear()
        self.channel_topic_field.addItems(
            self._topic_message_types
        )
        self.channel_topic_field.setEditText(current_topic)
        self.channel_topic_field.blockSignals(False)
        self._refresh_message_type_suggestions(
            current_topic,
            select_advertised=False,
        )

    def apply_sensor_head_connection(self, state) -> None:
        """Render discovered IDs without constraining manual creation."""
        self._head_connection_state = state
        selected_id = self.detected_mount_dropdown.currentData() or ""
        connected_ids = (
            tuple(state.connected_sensor_ids) if state is not None else ()
        )
        self.detected_mount_dropdown.blockSignals(True)
        self.detected_mount_dropdown.clear()
        for sensor_id in connected_ids:
            self.detected_mount_dropdown.addItem(sensor_id, sensor_id)
        if connected_ids:
            index = self.detected_mount_dropdown.findData(selected_id)
            self.detected_mount_dropdown.setCurrentIndex(
                index if index >= 0 else 0
            )
        else:
            self.detected_mount_dropdown.addItem(
                "No connected sensor heads",
                "",
            )
        self.detected_mount_dropdown.blockSignals(False)
        self._render_head_connection_state()
        self._refresh_detected_mount_actions()

    def edit_selected_sensor(self) -> None:
        """Open the selected stored sensor in the definition form."""
        definition = self._selected_definition()
        if definition is None or not self._mutation_allowed(
            definition.sensor_id
        ):
            return
        self._editing_sensor_id = definition.sensor_id
        self.mount_id_field.setText(definition.sensor_id)
        self.mount_id_field.setReadOnly(True)
        self.display_name_field.setText(definition.display_name)
        for field, value in zip(
            self.translation_fields,
            definition.position,
        ):
            field.setText(self._format_number(value))
        for field, value in zip(
            self.rotation_fields,
            definition.rotation_degrees,
        ):
            field.setText(self._format_number(value))
        self._set_channel_rows(definition.channels)
        self.transform_status_value.setText(
            "Editing manual transform. Saving overwrites this sensor."
        )
        self._set_transform_fields_editable(True)
        self._refresh_save_availability()
        self._refresh_detected_mount_actions()

    def clear_sensor_definition(self) -> None:
        """Reset the definition form to create a new sensor mount."""
        self._editing_sensor_id = ""
        self.mount_id_field.setReadOnly(False)
        self.mount_id_field.clear()
        self.display_name_field.clear()
        self.probe_frame_field.clear()
        for field in (
            *self.translation_fields,
            *self.rotation_fields,
        ):
            field.setText("0.0")
        self._set_channel_rows((_DEFAULT_GEOMETRY_CHANNEL,))
        self._set_transform_fields_editable(True)
        self._set_channel_fields_editable(True)
        self.save_mount_button.setEnabled(True)
        self.discard_mount_button.setEnabled(True)
        self.transform_status_value.setText(
            "Manual transform. Enter a new mount ID and save."
        )
        self.mount_id_field.setFocus()
        self._refresh_detected_mount_actions()

    def mark_sensor_save_pending(self) -> None:
        """Prevent duplicate submissions while the registry is saving."""
        self.mount_id_field.setReadOnly(True)
        self._set_transform_fields_editable(False)
        self._set_channel_fields_editable(False)
        self.save_mount_button.setEnabled(False)
        self.discard_mount_button.setEnabled(False)
        self.transform_status_value.setText(
            "Saving sensor transform..."
        )
        self._refresh_detected_mount_actions()

    def finish_sensor_save(
        self,
        success: bool,
        message: str,
    ) -> None:
        """Render the result of one registry create or update request."""
        detail = message.strip() or (
            "Sensor mount saved"
            if success
            else "Sensor mount save failed"
        )
        if success:
            self.clear_sensor_definition()
            self.transform_status_value.setText(detail)
            return
        self.mount_id_field.setReadOnly(bool(self._editing_sensor_id))
        self._set_transform_fields_editable(True)
        self._set_channel_fields_editable(True)
        self.discard_mount_button.setEnabled(True)
        self.transform_status_value.setText(detail)
        self._refresh_save_availability()
        self._refresh_detected_mount_actions()

    def finish_sensor_deletion(
        self,
        sensor_id: str,
        success: bool,
        message: str,
    ) -> None:
        """Render one deletion result and clear matching edit state."""
        detail = message.strip() or (
            "Sensor deleted"
            if success
            else "Sensor deletion failed"
        )
        if success and self._editing_sensor_id == sensor_id:
            self.clear_sensor_definition()
        self.transform_status_value.setText(detail)
        self._refresh_registry_actions()

    def _render_attachment_state(self) -> None:
        state = self._attachment_state
        if state is None:
            self.attachment_name_value.setText("—")
            self.attachment_status_value.setText("Waiting for state")
            self.attachment_probe_frame_value.setText("—")
            self.clear_attachment_button.setEnabled(False)
            return

        status_name = getattr(state.status, "value", str(state.status))
        selected_id = state.selected_sensor_id
        if not selected_id:
            display_name = "No sensor"
            probe_frame = "hand"
        else:
            definition = self._definitions.get(selected_id)
            display_name = (
                definition.display_name
                if definition is not None
                else selected_id
            )
            probe_frame = (
                definition.probe_frame
                if definition is not None
                else "—"
            )

        status_text = (
            "Confirmation pending"
            if status_name in {"none", "pending"}
            else "Confirmed"
        )
        self.attachment_name_value.setText(display_name)
        self.attachment_status_value.setText(status_text)
        self.attachment_probe_frame_value.setText(probe_frame)
        self.clear_attachment_button.setEnabled(
            bool(selected_id)
            or status_name == "pending"
        )

        if selected_id:
            index = self.active_mount_dropdown.findData(selected_id)
            if index >= 0:
                self.active_mount_dropdown.setCurrentIndex(index)

    def _render_head_connection_state(self) -> None:
        state = self._head_connection_state
        if state is None:
            self.head_connection_value.setText("Status unavailable")
            self.head_connection_value.setToolTip("")
            return
        status_name = getattr(state.status, "value", str(state.status))
        connected = tuple(state.connected_sensor_ids)
        values = {
            "unknown": "Status unavailable",
            "agent_unavailable": "Agent unavailable",
            "no_heads": "Offline" if state.expected_sensor_id else "None",
            "unassigned": "Unassigned: " + ", ".join(connected),
            "matched": f"Connected: {state.expected_sensor_id}",
            "mismatch": "Mismatch: " + ", ".join(connected),
        }
        self.head_connection_value.setText(
            values.get(status_name, "Status unavailable")
        )
        self.head_connection_value.setToolTip(state.detail)

    def _use_detected_mount_id(self) -> None:
        if self._editing_sensor_id:
            return
        sensor_id = self.detected_mount_dropdown.currentData() or ""
        if not sensor_id:
            return
        self.mount_id_field.setText(sensor_id)
        self.transform_status_value.setText(
            "Detected head ID copied. Enter its display name and transform."
        )

    def _refresh_detected_mount_actions(self) -> None:
        has_id = bool(self.detected_mount_dropdown.currentData() or "")
        enabled = (
            has_id
            and not self._editing_sensor_id
            and not self.mount_id_field.isReadOnly()
        )
        self.detected_mount_dropdown.setEnabled(enabled)
        self.use_detected_mount_button.setEnabled(enabled)

    def _request_selected_mount(self) -> None:
        sensor_id = self.active_mount_dropdown.currentData() or ""
        if sensor_id:
            self.select_requested.emit(sensor_id)

    def _request_no_sensor(self) -> None:
        self.select_requested.emit("")

    def _request_sensor_save(self) -> None:
        intent = self._definition_intent()
        if intent is None:
            return
        if self._editing_sensor_id:
            if not self._mutation_allowed(self._editing_sensor_id):
                self.transform_status_value.setText(
                    "Remove or switch the selected sensor before editing it."
                )
                self._refresh_save_availability()
                return
            self.update_requested.emit(intent)
            return
        if intent.sensor_id in self._definitions:
            self.transform_status_value.setText(
                f"Mount ID '{intent.sensor_id}' already exists. "
                "Select it and use Edit Sensor."
            )
            return
        self.create_requested.emit(intent)

    def _request_sensor_deletion(self) -> None:
        definition = self._selected_definition()
        if definition is None or not self._mutation_allowed(
            definition.sensor_id
        ):
            return
        self.delete_requested.emit(definition.sensor_id)

    def _definition_intent(self):
        sensor_id = self.mount_id_field.text().strip()
        display_name = self.display_name_field.text().strip()
        if not sensor_id:
            self.transform_status_value.setText(
                "Mount ID must not be empty."
            )
            return None
        if not display_name:
            self.transform_status_value.setText(
                "Display name must not be empty."
            )
            return None
        try:
            translation = tuple(
                self._field_value(field)
                for field in self.translation_fields
            )
            rotation = tuple(
                self._field_value(field)
                for field in self.rotation_fields
            )
        except ValueError as exception:
            self.transform_status_value.setText(str(exception))
            return None
        return SensorDefinitionIntent(
            sensor_id=sensor_id,
            display_name=display_name,
            translation_m=translation,
            rotation_degrees=rotation,
            channels=self._channel_values(),
        )

    def _refresh_registry_actions(self) -> None:
        definition = self._selected_definition()
        enabled = (
            definition is not None
            and self._mutation_allowed(definition.sensor_id)
        )
        self.edit_mount_button.setEnabled(enabled)
        self.delete_mount_button.setEnabled(enabled)
        selected_active = (
            definition is not None
            and not self._mutation_allowed(definition.sensor_id)
        )
        tooltip = (
            "Remove or switch this sensor before editing or deleting it."
            if selected_active
            else ""
        )
        self.edit_mount_button.setToolTip(tooltip)
        self.delete_mount_button.setToolTip(tooltip)

    def _refresh_save_availability(self) -> None:
        if not self._editing_sensor_id:
            self.save_mount_button.setEnabled(True)
            return
        allowed = self._mutation_allowed(self._editing_sensor_id)
        self.save_mount_button.setEnabled(allowed)
        if not allowed:
            self.transform_status_value.setText(
                "Remove or switch this sensor before editing it."
            )

    def _mutation_allowed(self, sensor_id: str) -> bool:
        state = self._attachment_state
        if state is None:
            return False
        return sensor_id != state.selected_sensor_id

    def _selected_definition(self):
        sensor_id = self._selected_table_sensor_id()
        return self._definitions.get(sensor_id)

    def _selected_table_sensor_id(self) -> str:
        row = self.mount_table.currentRow()
        if row < 0:
            return ""
        item = self.mount_table.item(row, 1)
        return item.text() if item is not None else ""

    def _restore_table_selection(self, sensor_id: str) -> None:
        if not sensor_id:
            return
        for row in range(self.mount_table.rowCount()):
            item = self.mount_table.item(row, 1)
            if item is not None and item.text() == sensor_id:
                self.mount_table.selectRow(row)
                return

    def _update_probe_frame_preview(self, sensor_id: str) -> None:
        normalized = sensor_id.strip()
        self.probe_frame_field.setText(
            f"{normalized}_probe" if normalized else ""
        )

    def _set_transform_fields_editable(self, editable: bool) -> None:
        self.display_name_field.setReadOnly(not editable)
        for field in (
            *self.translation_fields,
            *self.rotation_fields,
        ):
            field.setReadOnly(not editable)

    def _set_channel_fields_editable(self, editable: bool) -> None:
        self.channel_table.setEnabled(editable)
        self.channel_id_field.setReadOnly(not editable)
        self.channel_source_field.setEnabled(editable)
        self._apply_channel_source()
        self.save_channel_button.setEnabled(editable)
        self.new_channel_button.setEnabled(editable)
        self.remove_channel_button.setEnabled(
            editable and self.channel_table.currentRow() >= 0
        )

    def _set_channel_rows(self, channels) -> None:
        self.channel_table.setRowCount(0)
        for channel in channels:
            self._append_channel_row(
                channel.channel_id,
                getattr(channel, "source_kind", "ros_topic"),
                channel.topic,
                channel.message_type,
            )
        self._clear_channel_editor()

    def _append_channel_row(
        self,
        channel_id: str,
        source_kind: str,
        topic: str,
        message_type: str,
    ) -> None:
        source_kind = getattr(source_kind, "value", source_kind)
        row = self.channel_table.rowCount()
        self.channel_table.insertRow(row)
        self._set_channel_row(
            row,
            channel_id,
            source_kind,
            topic,
            message_type,
        )

    def _set_channel_row(
        self,
        row: int,
        channel_id: str,
        source_kind: str,
        topic: str,
        message_type: str,
    ) -> None:
        for column, value in enumerate(
            (
                channel_id,
                self._source_label(source_kind),
                topic,
                message_type,
            )
        ):
            item = self.channel_table.item(row, column)
            if item is None:
                item = QTableWidgetItem()
                self.channel_table.setItem(row, column, item)
            item.setText(value)
            item.setToolTip(value)
            if column == 1:
                item.setData(Qt.UserRole, source_kind)

    def _selected_channel_row(self) -> int:
        row = self.channel_table.currentRow()
        return row if 0 <= row < self.channel_table.rowCount() else -1

    def _populate_selected_channel(self) -> None:
        row = self._selected_channel_row()
        selected = row >= 0
        self.remove_channel_button.setEnabled(
            selected and self.channel_table.isEnabled()
        )
        self.save_channel_button.setText(
            "Update Channel" if selected else "Add Channel"
        )
        if not selected:
            return
        self.channel_id_field.setText(
            self.channel_table.item(row, 0).text()
        )
        source_kind = (
            self.channel_table.item(row, 1).data(Qt.UserRole)
            or "ros_topic"
        )
        source_index = self.channel_source_field.findData(source_kind)
        self.channel_source_field.setCurrentIndex(
            source_index if source_index >= 0 else 0
        )
        self.channel_topic_field.setEditText(
            self.channel_table.item(row, 2).text()
        )
        self.channel_message_type_field.setEditText(
            self.channel_table.item(row, 3).text()
        )

    def _clear_channel_editor(self) -> None:
        self.channel_table.clearSelection()
        self.channel_table.setCurrentCell(-1, -1)
        self.channel_id_field.clear()
        self.channel_source_field.setCurrentIndex(
            self.channel_source_field.findData("ros_topic")
        )
        self.channel_topic_field.setEditText("")
        self.channel_message_type_field.setEditText("")
        self.save_channel_button.setText("Add Channel")
        self.remove_channel_button.setEnabled(False)

    def _save_channel_row(self) -> None:
        channel_id = self.channel_id_field.text().strip()
        source_kind = self.channel_source_field.currentData()
        topic = self.channel_topic_field.currentText().strip()
        message_type = (
            self.channel_message_type_field.currentText().strip()
        )
        if source_kind == "spot_geometry":
            topic = ""
            message_type = ""
        values = (channel_id, source_kind, topic, message_type)
        required = (
            ((channel_id, "Channel ID"),)
            if source_kind == "spot_geometry"
            else (
                (channel_id, "Channel ID"),
                (topic, "ROS topic"),
                (message_type, "Message type"),
            )
        )
        for value, label in required:
            if not value:
                self.transform_status_value.setText(
                    f"{label} must not be empty."
                )
                return

        selected_row = self._selected_channel_row()
        for row in range(self.channel_table.rowCount()):
            if row == selected_row:
                continue
            if self.channel_table.item(row, 0).text() == values[0]:
                self.transform_status_value.setText(
                    f"Channel ID '{values[0]}' already exists."
                )
                return

        if selected_row < 0:
            self._append_channel_row(*values)
        else:
            self._set_channel_row(selected_row, *values)
        self.transform_status_value.setText(
            f"Configured channel '{channel_id}'."
        )
        self._clear_channel_editor()

    def _remove_selected_channel(self) -> None:
        row = self._selected_channel_row()
        if row < 0:
            return
        channel_id = self.channel_table.item(row, 0).text()
        self.channel_table.removeRow(row)
        self.transform_status_value.setText(
            f"Removed channel '{channel_id}' from this form."
        )
        self._clear_channel_editor()

    def _apply_topic_type_suggestion(self, topic: str) -> None:
        self._refresh_message_type_suggestions(
            topic,
            select_advertised=True,
        )

    def _apply_channel_source(self, _index=None) -> None:
        is_ros_topic = (
            self.channel_source_field.currentData() == "ros_topic"
        )
        editable = self.channel_table.isEnabled()
        self.channel_topic_field.setEnabled(editable and is_ros_topic)
        self.channel_message_type_field.setEnabled(
            editable and is_ros_topic
        )
        if not is_ros_topic:
            self.channel_topic_field.setEditText("")
            self.channel_message_type_field.setEditText("")

    def _refresh_message_type_suggestions(
        self,
        topic: str,
        select_advertised: bool,
    ) -> None:
        current_type = self.channel_message_type_field.currentText()
        advertised = self._topic_message_types.get(topic.strip(), ())
        all_types = tuple(
            sorted(
                {
                    message_type
                    for types in self._topic_message_types.values()
                    for message_type in types
                }
            )
        )
        values = tuple(dict.fromkeys((*advertised, *all_types)))
        self.channel_message_type_field.blockSignals(True)
        self.channel_message_type_field.clear()
        self.channel_message_type_field.addItems(values)
        if select_advertised and advertised:
            selected_type = (
                current_type
                if current_type in advertised
                else advertised[0]
            )
        else:
            selected_type = current_type
        self.channel_message_type_field.setEditText(selected_type)
        self.channel_message_type_field.blockSignals(False)

    def _channel_values(self) -> tuple:
        return tuple(
            SensorChannelIntent(
                channel_id=self.channel_table.item(row, 0).text(),
                source_kind=(
                    self.channel_table.item(row, 1).data(Qt.UserRole)
                    or "ros_topic"
                ),
                topic=self.channel_table.item(row, 2).text(),
                message_type=self.channel_table.item(row, 3).text(),
            )
            for row in range(self.channel_table.rowCount())
        )

    @staticmethod
    def _source_label(source_kind: str) -> str:
        return {
            "ros_topic": "ROS topic",
            "spot_geometry": "Spot geometry",
        }.get(source_kind, source_kind)

    def _append_definition_row(self, definition) -> None:
        row = self.mount_table.rowCount()
        self.mount_table.insertRow(row)
        values = (
            definition.display_name,
            definition.sensor_id,
            definition.probe_frame,
            "Manual",
        )
        for column, value in enumerate(values):
            self.mount_table.setItem(
                row,
                column,
                QTableWidgetItem(value),
            )

    @staticmethod
    def _field_value(field: QLineEdit) -> float:
        text = field.text().strip()
        if not text:
            raise ValueError("All transform values must be provided.")
        try:
            return float(text)
        except ValueError as exception:
            raise ValueError(
                f"Invalid transform value: {text}"
            ) from exception

    @staticmethod
    def _format_number(value: float) -> str:
        return format(float(value), ".8g")

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
            field.setText("0.0")
            field.setMaximumWidth(85)
            validator = QDoubleValidator(field)
            validator.setLocale(QLocale.c())
            validator.setNotation(QDoubleValidator.StandardNotation)
            field.setValidator(validator)
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
        transform_source: str,
    ) -> None:
        """Add one presentation-only row for layout evaluation."""
        row = self.mount_table.rowCount()
        self.mount_table.insertRow(row)
        for column, value in enumerate(
            (display_name, mount_id, probe_frame, transform_source)
        ):
            self.mount_table.setItem(
                row,
                column,
                QTableWidgetItem(value),
            )
        self.empty_registry_label.setVisible(False)


__all__ = [
    "SensorChannelIntent",
    "SensorControls",
    "SensorDefinitionIntent",
]
