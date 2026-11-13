"""Browse saved probe points and submit their execution or deletion intents."""

from uuid import uuid4

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPalette
from PyQt5.QtWidgets import (
    QDoubleSpinBox,
    QFrame,
    QGroupBox,
    QWidget,
    QHBoxLayout,
    QLabel,
    QListWidget,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
)
from fault_detector_msgs.msg import (
    ApplicationCommandState,
    OperationalIntent,
    ProbeSetupIntent,
    ProbeSetupState,
)


class SavedProbeControls(QFrame):
    """Own saved-point selection, input drafts, and request feedback."""

    def __init__(
        self, parent=None, *, submit_setup, execute_operation, ask_question,
        is_refinement_start_pending,
    ):
        super().__init__(parent)
        self._submit_setup = submit_setup
        self._execute_operation = execute_operation
        self._ask_question = ask_question
        self._is_refinement_start_pending = is_refinement_start_pending
        self._state = None
        self._scope = None
        self._operation_contexts = {}
        self._delete_point_pending = ""
        self._selection_after_finalization = None

        self.setFrameShape(QFrame.StyledPanel)
        outer = QHBoxLayout(self)
        outer.setContentsMargins(12, 12, 12, 12)
        outer.setSpacing(20)
        sidebar = QWidget()
        sidebar.setMinimumWidth(190)
        sidebar.setMaximumWidth(280)
        sidebar_layout = QVBoxLayout(sidebar)
        sidebar_layout.setContentsMargins(0, 0, 0, 0)
        details = QWidget()
        layout = QVBoxLayout(details)
        outer.addWidget(sidebar, 1)
        outer.addWidget(details, 3)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        title = QLabel("Probe Points")
        title_font = title.font()
        title_font.setBold(True)
        title.setFont(title_font)
        sidebar_layout.addWidget(title)

        self.overview_label = QLabel(
            "Select an inspection routine to add a probe point."
        )
        self.overview_label.setWordWrap(True)
        sidebar_layout.addWidget(self.overview_label)

        self.selection_label = QLabel(
            "Select a probe point"
        )
        selection_font = self.selection_label.font()
        selection_font.setBold(True)
        self.selection_label.setFont(selection_font)
        layout.addWidget(self.selection_label)

        self.points_list = QListWidget()
        palette = self.points_list.palette()
        highlight = palette.color(QPalette.Highlight).name()
        highlighted_text = palette.color(
            QPalette.HighlightedText
        ).name()
        self.points_list.setStyleSheet(
            "QListWidget::item {"
            " padding: 8px;"
            " border: 1px solid transparent;"
            "}"
            "QListWidget::item:selected {"
            f" background: {highlight};"
            f" color: {highlighted_text};"
            f" border: 2px solid {highlighted_text};"
            " font-weight: bold;"
            "}"
        )
        self.points_list.currentRowChanged.connect(
            self._selection_changed
        )
        sidebar_layout.addWidget(self.points_list, 1)

        self.delete_button = QPushButton(
            "Delete"
        )
        self.delete_button.setEnabled(False)
        self.delete_button.setToolTip(
            "Delete the selected probe point including its stored safe "
            "approach, aligned pre-approach, and probe target data."
        )
        self.delete_button.clicked.connect(
            self.handle_delete
        )
        self.point_tools = QHBoxLayout()
        self.point_tools.addWidget(self.delete_button)
        sidebar_layout.addLayout(self.point_tools)
        self.sidebar_layout = sidebar_layout

        self.target_distance = QDoubleSpinBox()
        self.target_distance.setDecimals(3)
        self.target_distance.setSuffix(" m")
        self.target_distance.setMaximumWidth(125)
        self.target_distance.setRange(0.0, 10.0)
        self.target_distance.setSingleStep(0.005)
        self.target_distance.setEnabled(False)
        self.target_distance.setToolTip(
            "Defaults to the selected point’s saved value. "
            "Changes apply only to this movement."
        )
        movement = QGroupBox("1  ·  Position the probe")
        movement_layout = QVBoxLayout(movement)
        movement_layout.setSpacing(8)
        layout.addWidget(movement)
        self.action_buttons = {}
        for label, operation in (
            (
                "Follow pre-approach path",
                OperationalIntent.INTENT_MOVE_SAVED_PROBE_ALIGNED_PREAPPROACH,
            ),
            (
                "Approach surface",
                OperationalIntent.INTENT_MOVE_SAVED_PROBE_CLOSE_TO_SURFACE,
            ),
        ):
            button = QPushButton(label)
            button.setEnabled(False)
            button.clicked.connect(
                lambda _checked=False, operation=operation: self.handle_motion(operation)
            )
            self.action_buttons[operation] = button
            if operation == OperationalIntent.INTENT_MOVE_SAVED_PROBE_CLOSE_TO_SURFACE:
                wall_row = QHBoxLayout()
                wall_row.addWidget(button, 1)
                wall_row.addWidget(QLabel("Surface distance:"))
                wall_row.addWidget(self.target_distance)
                movement_layout.addLayout(wall_row)
            else:
                movement_layout.addWidget(button)
        self.custom_probe_button = QPushButton("Move to probe point")
        self.custom_probe_button.setEnabled(False)
        self.custom_probe_button.clicked.connect(
            lambda: self.handle_motion(
                OperationalIntent.INTENT_MOVE_SAVED_CUSTOM_PROBE_PATH,
            )
        )
        movement_layout.addWidget(self.custom_probe_button)
        self.movement_hint = QLabel("Select a probe point to see its movement options.")
        self.movement_hint.setWordWrap(True)
        self.movement_hint.setMinimumHeight(self.movement_hint.fontMetrics().lineSpacing() * 3)
        movement_layout.addWidget(self.movement_hint)
        recording = QGroupBox("2  ·  Execute and record")
        recording_layout = QVBoxLayout(recording)
        recording_layout.setSpacing(10)
        layout.addWidget(recording)
        recording_row = QHBoxLayout()
        self.record_duration = QDoubleSpinBox()
        self.record_duration.setMaximumWidth(125)
        self.record_duration.setRange(0.1, 3600.0)
        self.record_duration.setValue(5.0)
        self.record_duration.setSuffix(" s")
        self.record_retries = QSpinBox()
        self.record_retries.setMaximumWidth(85)
        self.record_retries.setRange(0, 100)
        self.record_retries.setToolTip(
            "Shared retry budget: return to the last successful goal "
            "and retry the failed step."
        )
        recording_row.addWidget(QLabel("Recording duration:"))
        recording_row.addWidget(self.record_duration)
        recording_row.addWidget(QLabel("Retries:"))
        recording_row.addWidget(self.record_retries)
        recording_row.addStretch()
        recording_layout.addLayout(recording_row)
        execute_button = QPushButton("Execute && Record")
        execute_button.setEnabled(False)
        execute_button.clicked.connect(lambda: self.handle_motion(
            OperationalIntent.INTENT_EXECUTE_PROBE_POINT,
        ))
        self.action_buttons[OperationalIntent.INTENT_EXECUTE_PROBE_POINT] = execute_button
        execute_button.setMinimumHeight(38)
        execute_button.setStyleSheet(
            "QPushButton:enabled { background: palette(highlight); color: palette(highlighted-text); "
            "font-weight: bold; border: none; border-radius: 4px; padding: 8px 16px; }"
        )
        recording_layout.addWidget(execute_button)
        self.status_label = QLabel("Select a saved probe point.")
        self.status_label.setWordWrap(True)
        status = QGroupBox("Execution status")
        status_layout = QVBoxLayout(status)
        self.status_label.setMinimumHeight(38)
        self.status_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        status_layout.addWidget(self.status_label)
        layout.addWidget(status)
        layout.addStretch()

    def attach_refinement_controls(self, start_button, abort_button):
        """Place the parent's refinement controls below the saved-point actions."""
        start_button.setText("+ Add")
        start_button.setToolTip("Add a new probe point")
        self.point_tools.insertWidget(0, start_button, 1)
        self.sidebar_layout.addWidget(abort_button)

    def apply_setup_state(self, state):
        """Render the authoritative snapshot while preserving selection drafts."""
        self._state = state
        deletion_point_id = self._delete_point_pending
        self._update_points(state)

        if (
            deletion_point_id
            and state.operation
            == ProbeSetupIntent.OPERATION_DELETE_PROBE_POINT
            and state.state in (
                ProbeSetupState.STATE_SUCCEEDED,
                ProbeSetupState.STATE_FAILED,
            )
        ):
            self._delete_point_pending = ""
            if state.state == ProbeSetupState.STATE_SUCCEEDED:
                self.status_label.setText(
                    f"{deletion_point_id}: deleted."
                )
            else:
                detail = (
                    state.detail.strip()
                    or state.validation_error.strip()
                    or "Delete failed."
                )
                self.status_label.setText(
                    f"{deletion_point_id}: delete failed: {detail}"
                )
            self._refresh_actions()

        if not state.selected_routine_id:
            overview = "Select an inspection routine to add a probe point."
        elif state.probe_point_ids:
            overview = ""
        else:
            overview = "No probe points are saved for this routine."
        self.overview_label.setText(overview)
        self.overview_label.setVisible(bool(overview))

    def select_after_finalization(self, object_id, routine_id, point_id):
        """Select a completed point when its matching snapshot is rendered."""
        self._selection_after_finalization = (object_id, routine_id, point_id)

    def clear_pending_selection(self):
        self._selection_after_finalization = None

    def _selection_changed(self, _row=None):
        state = self._state
        current = self.points_list.currentItem()
        self.selection_label.setText(
            (
                f"Probe point: {current.text()}"
                if current is not None
                else "Select a probe point"
            )
        )
        row = self.points_list.currentRow()
        if (
            state is not None
            and 0 <= row < len(state.probe_point_target_surface_distances_m)
        ):
            self.target_distance.setValue(
                state.probe_point_target_surface_distances_m[row]
            )
        else:
            self.target_distance.setValue(0.0)
        if not self._operation_contexts:
            self.status_label.setText("Ready to move or execute." if current else "Select a saved probe point.")
        self._refresh_actions()

    def _refresh_actions(self):
        state = self._state
        enabled = bool(
            state is not None
            and state.selected_object_id
            and state.selected_routine_id
            and self.points_list.currentItem() is not None
            and not state.refinement_active
            and not state.motion_pending
            and not self._is_refinement_start_pending()
            and not self._delete_point_pending
        )
        row = self.points_list.currentRow()
        custom = bool(
            state is not None
            and 0 <= row < len(state.probe_point_fully_custom)
            and state.probe_point_fully_custom[row]
        )
        self.target_distance.setEnabled(
            enabled and not custom
            and 0 <= row < len(state.probe_point_target_surface_distances_m)
        )
        self.record_duration.setEnabled(enabled)
        self.record_retries.setEnabled(enabled)
        for button in self.action_buttons.values():
            button.setEnabled(enabled)
        surface_button = self.action_buttons[
            OperationalIntent.INTENT_MOVE_SAVED_PROBE_CLOSE_TO_SURFACE
        ]
        surface_button.setEnabled(enabled and not custom)
        self.custom_probe_button.setEnabled(enabled and custom)
        self.movement_hint.setText(
            "Custom point: use the saved path. Surface approach is unavailable." if custom else
            "Surface-relative point: use the surface approach. Custom-path movement is unavailable."
            if self.points_list.currentItem() is not None else
            "Select a probe point to enable movement and recording."
        )
        surface_button.setToolTip("Available for surface-relative probe points.")
        self.custom_probe_button.setToolTip(
            "Available for fully custom probe points; follows the saved final path."
        )
        self.delete_button.setEnabled(
            enabled and not self._operation_contexts
        )
        self.points_list.setEnabled(
            not bool(self._delete_point_pending)
        )

    def _update_points(self, state):
        scope = (state.selected_object_id, state.selected_routine_id)
        ids = list(state.probe_point_ids) if all(scope) else []
        current = self.points_list.currentItem()
        selected = (
            current.text()
            if current and scope == self._scope
            else None
        )
        completed = self._selection_after_finalization
        select_completed = bool(
            completed is not None
            and completed[:2] == scope
            and completed[2] in ids
        )
        if select_completed:
            selected = completed[2]
        previous = [
            self.points_list.item(i).text()
            for i in range(self.points_list.count())
        ]
        changed = scope != self._scope or ids != previous
        if changed:
            self.points_list.blockSignals(True)
            self.points_list.clear()
            self.points_list.addItems(ids)
            if selected in ids:
                self.points_list.setCurrentRow(
                    ids.index(selected)
                )
            self.points_list.blockSignals(False)
            self._scope = scope

        if select_completed:
            if not changed:
                self.points_list.blockSignals(True)
                self.points_list.setCurrentRow(
                    ids.index(selected)
                )
                self.points_list.blockSignals(False)
            self._selection_after_finalization = None
            self._selection_changed()
            if not self._operation_contexts:
                self.status_label.setText(
                    f"{selected}: ready."
                )
        elif changed:
            if selected not in ids:
                self._selection_changed()
            if not self._operation_contexts:
                self.status_label.setText(
                    "Select a saved probe point."
                )
        self._refresh_actions()

    def handle_delete(self):
        self._refresh_actions()
        if not self.delete_button.isEnabled():
            return False

        state = self._state
        current = self.points_list.currentItem()
        if state is None or current is None:
            return False

        point_id = current.text()
        scope = (
            state.selected_object_id,
            state.selected_routine_id,
        )
        if (
            self._scope != scope
            or point_id not in state.probe_point_ids
        ):
            return False

        if not self._ask_question(
            "Delete Probe Point",
            (
                f"Delete probe point '{point_id}' from "
                f"'{scope[0]}/{scope[1]}'?\n\n"
                "Its saved approach and aligned positions are deleted "
                "with the probe point."
            ),
        ):
            return False

        intent = ProbeSetupIntent()
        intent.operation = (
            ProbeSetupIntent.OPERATION_DELETE_PROBE_POINT
        )
        intent.object_id = scope[0]
        intent.routine_id = scope[1]
        intent.probe_point_id = point_id

        self._delete_point_pending = point_id
        self.status_label.setText(
            f"{point_id}: deleting..."
        )
        self._refresh_actions()
        request_id = self._submit_setup(intent)
        if request_id is None:
            self._delete_point_pending = ""
            self.status_label.setText(
                f"{point_id}: delete request could not be submitted."
            )
            self._refresh_actions()
            return False
        return True

    def handle_motion(self, operation):
        self._refresh_actions()
        if operation == OperationalIntent.INTENT_MOVE_SAVED_CUSTOM_PROBE_PATH:
            button = self.custom_probe_button
        else:
            button = self.action_buttons.get(operation)
        if button is None or not button.isEnabled():
            return False
        state = self._state
        point_id = self.points_list.currentItem().text()
        if (
            self._scope != (state.selected_object_id, state.selected_routine_id)
            or point_id not in state.probe_point_ids
        ):
            return False
        intent = OperationalIntent()
        intent.intent = operation
        intent.object_id = state.selected_object_id
        intent.routine_id = state.selected_routine_id
        intent.probe_point_id = point_id
        if operation == OperationalIntent.INTENT_EXECUTE_PROBE_POINT:
            intent.duration_sec = self.record_duration.value()
            intent.retries = self.record_retries.value()
        if (
            operation == OperationalIntent.INTENT_MOVE_SAVED_PROBE_CLOSE_TO_SURFACE
            and self.target_distance.isEnabled()
        ):
            intent.override_target_surface_distance = True
            intent.target_surface_distance_m = self.target_distance.value()
        context_id = "saved-probe-" + uuid4().hex
        self._operation_contexts[context_id] = point_id
        self.status_label.setText(f"{point_id}: submitting movement...")
        self._refresh_actions()
        request_id = self._execute_operation(
            intent, context_id=context_id,
        )
        if request_id is None:
            self.handle_operation_rejected(
                "Movement could not be submitted.", context_id
            )
            return False
        return True

    def handle_operation_rejected(self, detail, context_id):
        if context_id not in self._operation_contexts:
            return
        point_id = self._operation_contexts.pop(context_id)
        self.status_label.setText(f"{point_id}: {detail}")
        self._refresh_actions()

    def handle_application_state(self, status):
        if status.context_id not in self._operation_contexts:
            return
        point_id = self._operation_contexts[status.context_id]
        detail = status.detail or {
            ApplicationCommandState.STATE_QUEUED: "Movement queued",
            ApplicationCommandState.STATE_UNKNOWN: "Movement outcome unknown; reconciling",
            ApplicationCommandState.STATE_STOP_REQUESTED: "Stop requested; awaiting confirmation",
            ApplicationCommandState.STATE_STOP_UNCONFIRMED: "Stop unconfirmed",
        }.get(status.state, "Movement in progress")
        if detail.startswith("BT executing "):
            command = detail.removeprefix("BT executing ")
            detail = {
                "move_to_tag": "Moving to the saved target…",
                "move_arm_relative": "Moving the arm…",
                "execute_probe_point": "Executing probe point and recording…",
            }.get(command, "Running: " + command.replace("_", " "))
        self.status_label.setToolTip(status.detail)
        self.status_label.setText(f"{point_id}: {detail}")
        if status.state in {
            ApplicationCommandState.STATE_SUCCEEDED,
            ApplicationCommandState.STATE_FAILED,
            ApplicationCommandState.STATE_CANCELLED,
            ApplicationCommandState.STATE_STOP_CONFIRMED,
        }:
            self._operation_contexts.pop(status.context_id)
            labels = {
                ApplicationCommandState.STATE_SUCCEEDED: "Movement completed",
                ApplicationCommandState.STATE_FAILED: "Movement failed",
                ApplicationCommandState.STATE_CANCELLED: "Movement cancelled",
                ApplicationCommandState.STATE_STOP_CONFIRMED: "Movement stop confirmed",
            }
            self.status_label.setText(
                f"{point_id}: " + labels[status.state]
                + (": " + status.detail if status.detail else "")
            )
        self._refresh_actions()

    def handle_operation_uncertain(self, detail, context_id):
        """Report lost transport feedback without releasing motion ownership."""
        text = f"Outcome unknown; awaiting confirmation: {detail}"
        point_id = self._operation_contexts.get(context_id)
        if point_id is not None:
            self.status_label.setText(f"{point_id}: {text}")
