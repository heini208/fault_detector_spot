"""Present saved routine navigation and submit setup or operational intent."""

from uuid import uuid4

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QComboBox,
    QGridLayout,
    QGroupBox,
    QLabel,
    QPushButton,
)
from fault_detector_msgs.msg import (
    ApplicationCommandState,
    OperationalIntent,
    ProbeSetupIntent,
    ProbeSetupState,
)


class RoutineNavigationControls(QGroupBox):
    """Keep navigation drafts separate from authoritative routine assignments."""

    def __init__(self, parent=None, *, submit_setup=None, execute_operation=None):
        super().__init__("Routine Navigation", parent)
        self._submit_setup = submit_setup
        self._execute_operation = execute_operation
        self._scope = ("", "")
        self._map_names = ()
        self._saved_map_name = ""
        self._saved_waypoint_name = ""
        self._waypoint_names = ()
        self._setup_blocked = False
        self._pending_save = None
        self._operation_contexts = {}

        layout = QGridLayout(self)
        layout.setColumnStretch(1, 1)
        self.map_dropdown = QComboBox()
        self.waypoint_dropdown = QComboBox()
        self.map_dropdown.setMinimumWidth(180)
        self.waypoint_dropdown.setMinimumWidth(180)
        self.set_map_button = QPushButton("Set Map")
        self.launch_map_button = QPushButton("Launch Map")
        self.set_waypoint_button = QPushButton("Set Waypoint")
        self.move_to_waypoint_button = QPushButton("Move to Waypoint")
        self.map_name_label = QLabel()
        self.waypoint_name_label = QLabel()
        for label in (self.map_name_label, self.waypoint_name_label):
            label.setFixedWidth(170)
            label.setWordWrap(True)
            label.setTextFormat(Qt.PlainText)
            self._show_assigned_name(label, "")
        for row, label, dropdown, save_button, name_label, action_button in (
            (0, "Map:", self.map_dropdown, self.set_map_button,
             self.map_name_label, self.launch_map_button),
            (1, "Waypoint:", self.waypoint_dropdown, self.set_waypoint_button,
             self.waypoint_name_label, self.move_to_waypoint_button),
        ):
            layout.addWidget(QLabel(label), row, 0)
            layout.addWidget(dropdown, row, 1)
            layout.addWidget(save_button, row, 2)
            layout.addWidget(name_label, row, 3)
            layout.addWidget(action_button, row, 4)
            for button in (save_button, action_button):
                button.setEnabled(False)
        self.set_map_button.setToolTip("Save this routine's map. Clearing or changing it clears the saved waypoint.")
        self.set_waypoint_button.setToolTip("Save the selected waypoint from this routine's saved map.")
        self.launch_map_button.setToolTip("Queue localization with this routine's saved map.")
        self.move_to_waypoint_button.setToolTip("Queue movement to this routine's saved waypoint.")

        self.status_label = QLabel("Select a routine to configure navigation.")
        self.status_label.setWordWrap(True)
        self.status_label.setTextFormat(Qt.PlainText)
        layout.addWidget(self.status_label, 2, 0, 1, 5)
        self.map_dropdown.currentIndexChanged.connect(self._refresh_waypoints)
        self.waypoint_dropdown.currentIndexChanged.connect(self._refresh_buttons)
        self.set_map_button.clicked.connect(self.handle_set_map)
        self.set_waypoint_button.clicked.connect(self.handle_set_waypoint)
        self.launch_map_button.clicked.connect(self.handle_launch_map)
        self.move_to_waypoint_button.clicked.connect(self.handle_move_to_waypoint)
        self._refresh_maps("")

    def apply_setup_state(self, state):
        """Render assignments and map-qualified choices from probe setup."""
        scope = (state.selected_object_id, state.selected_routine_id)
        scope_changed = scope != self._scope
        selected_map = self.map_dropdown.currentData() or ""
        selected_waypoint = self.waypoint_dropdown.currentData() or ""
        old_saved_map = self._saved_map_name
        old_saved_waypoint = self._saved_waypoint_name
        self._scope = scope
        self._map_names = tuple(state.navigation_map_names)
        self._saved_map_name = state.routine_map_name if all(scope) else ""
        self._saved_waypoint_name = state.routine_waypoint_name if all(scope) else ""
        self._waypoint_names = tuple(state.routine_waypoint_names)
        self._setup_blocked = bool(
            state.refinement_active or state.motion_pending
            or state.state in {ProbeSetupState.STATE_QUEUED, ProbeSetupState.STATE_RUNNING}
        )
        if scope_changed or selected_map == old_saved_map:
            selected_map = self._saved_map_name
        if (scope_changed or old_saved_map != self._saved_map_name
                or selected_waypoint == old_saved_waypoint):
            selected_waypoint = self._saved_waypoint_name

        pending = self._pending_save
        if (pending is not None and state.operation == pending[0]
                and state.state in {
                    ProbeSetupState.STATE_SUCCEEDED,
                    ProbeSetupState.STATE_FAILED,
                    ProbeSetupState.STATE_CANCELLED,
                }):
            self._pending_save = None
            if scope == pending[1]:
                self.status_label.setText(state.detail or (
                    "Navigation assignment saved."
                    if state.state == ProbeSetupState.STATE_SUCCEEDED
                    else "Navigation assignment was not saved."
                ))
        elif scope_changed:
            self.status_label.setText(
                "Select a map or waypoint, then save the assignment."
                if all(scope) else "Select a routine to configure navigation."
            )
        self._show_assigned_name(self.map_name_label, self._saved_map_name)
        self._show_assigned_name(self.waypoint_name_label, self._saved_waypoint_name)
        self._refresh_maps(selected_map, selected_waypoint)

    def _refresh_maps(self, selected, selected_waypoint=None):
        self.map_dropdown.blockSignals(True)
        self.map_dropdown.clear()
        self.map_dropdown.addItem("Not set", "")
        for name in self._map_names:
            self.map_dropdown.addItem(name, name)
        if self._saved_map_name and self._saved_map_name not in self._map_names:
            self.map_dropdown.addItem(self._saved_map_name, self._saved_map_name)
        self.map_dropdown.setCurrentIndex(max(0, self.map_dropdown.findData(selected)))
        self.map_dropdown.blockSignals(False)
        self._refresh_waypoints(selected_waypoint=selected_waypoint)

    def _refresh_waypoints(self, _index=None, *, selected_waypoint=None):
        selected_map = self.map_dropdown.currentData() or ""
        if selected_waypoint is None:
            selected_waypoint = (
                self.waypoint_dropdown.currentData()
                if self.waypoint_dropdown.property("map_id") == selected_map
                else self._saved_waypoint_name
                if selected_map == self._saved_map_name else ""
            )
        available = bool(selected_map) and selected_map == self._saved_map_name
        if not selected_map:
            placeholder = "Save a map first"
        elif not available:
            placeholder = "Save the selected map first"
        else:
            placeholder = "Not set"
        self.waypoint_dropdown.blockSignals(True)
        self.waypoint_dropdown.clear()
        self.waypoint_dropdown.addItem(placeholder, "")
        if available:
            for name in self._waypoint_names:
                self.waypoint_dropdown.addItem(name, name)
            if (self._saved_waypoint_name
                    and self._saved_waypoint_name not in self._waypoint_names):
                self.waypoint_dropdown.addItem(self._saved_waypoint_name, self._saved_waypoint_name)
        self.waypoint_dropdown.setCurrentIndex(max(0, self.waypoint_dropdown.findData(selected_waypoint)))
        self.waypoint_dropdown.setProperty("map_id", selected_map)
        self.waypoint_dropdown.blockSignals(False)
        self.waypoint_dropdown.setToolTip(
            "Save the selected map before assigning one of its waypoints."
            if selected_map and not available else ""
        )
        self._refresh_buttons()

    def _refresh_buttons(self, _index=None):
        ready = bool(all(self._scope) and not self._setup_blocked)
        editable = ready and self._pending_save is None
        selected_map = self.map_dropdown.currentData() or ""
        selected_waypoint = self.waypoint_dropdown.currentData() or ""
        saved_map_selected = bool(self._saved_map_name) and selected_map == self._saved_map_name
        self.map_dropdown.setEnabled(editable and bool(self._map_names or self._saved_map_name))
        self.waypoint_dropdown.setEnabled(
            editable and saved_map_selected
            and bool(self._waypoint_names or self._saved_waypoint_name)
        )
        self.set_map_button.setEnabled(
            editable and self._submit_setup is not None
            and selected_map != self._saved_map_name
        )
        self.set_waypoint_button.setEnabled(
            editable and self._submit_setup is not None and saved_map_selected
            and selected_waypoint != self._saved_waypoint_name
        )
        self.launch_map_button.setEnabled(
            editable and self._execute_operation is not None and bool(self._saved_map_name)
        )
        self.move_to_waypoint_button.setEnabled(
            editable and self._execute_operation is not None
            and bool(self._saved_map_name and self._saved_waypoint_name)
        )

    def handle_set_map(self):
        return self._save_assignment(ProbeSetupIntent.OPERATION_SAVE_ROUTINE_MAP)

    def handle_set_waypoint(self):
        return self._save_assignment(ProbeSetupIntent.OPERATION_SAVE_ROUTINE_WAYPOINT)

    def _save_assignment(self, operation):
        self._refresh_buttons()
        is_map = operation == ProbeSetupIntent.OPERATION_SAVE_ROUTINE_MAP
        button = self.set_map_button if is_map else self.set_waypoint_button
        if not button.isEnabled():
            return False
        intent = ProbeSetupIntent()
        intent.operation = operation
        intent.object_id, intent.routine_id = self._scope
        intent.map_name = (self.map_dropdown.currentData() or "") if is_map else self._saved_map_name
        if not is_map:
            intent.waypoint_name = self.waypoint_dropdown.currentData() or ""
        self._pending_save = (operation, self._scope)
        self.status_label.setText("Saving navigation assignment...")
        self._refresh_buttons()
        if self._submit_setup(intent) is None:
            self.handle_setup_rejected("Navigation assignment could not be submitted.")
            return False
        return True

    def handle_setup_rejected(self, detail, operation=None):
        if self._pending_save is None or (
            operation is not None and operation != self._pending_save[0]
        ):
            return
        self._pending_save = None
        self.status_label.setText(detail)
        self._refresh_buttons()

    def handle_launch_map(self):
        return self._submit_operation(OperationalIntent.INTENT_LAUNCH_ROUTINE_MAP)

    def handle_move_to_waypoint(self):
        return self._submit_operation(OperationalIntent.INTENT_MOVE_TO_ROUTINE_WAYPOINT)

    def _submit_operation(self, operation):
        self._refresh_buttons()
        launch = operation == OperationalIntent.INTENT_LAUNCH_ROUTINE_MAP
        button = self.launch_map_button if launch else self.move_to_waypoint_button
        if not button.isEnabled():
            return False
        intent = OperationalIntent()
        intent.intent = operation
        intent.object_id, intent.routine_id = self._scope
        context_id = "routine-navigation-" + uuid4().hex
        self._operation_contexts[context_id] = self._scope
        self.status_label.setText("Submitting map launch..." if launch else "Submitting waypoint movement...")
        if self._execute_operation(intent, context_id=context_id) is None:
            self.handle_operation_rejected("Navigation command could not be submitted.", context_id)
            return False
        return True

    def handle_operation_rejected(self, detail, context_id):
        scope = self._operation_contexts.pop(context_id, None)
        if scope == self._scope:
            self.status_label.setText(detail)

    def handle_application_state(self, state):
        scope = self._operation_contexts.get(state.context_id)
        if scope is None:
            return
        labels = {
            ApplicationCommandState.STATE_QUEUED: "Navigation command queued.",
            ApplicationCommandState.STATE_DISPATCHED: "Navigation command dispatched.",
            ApplicationCommandState.STATE_RUNNING: "Navigation command running.",
            ApplicationCommandState.STATE_SUCCEEDED: "Navigation command completed.",
            ApplicationCommandState.STATE_FAILED: "Navigation command failed.",
            ApplicationCommandState.STATE_CANCELLED: "Navigation command cancelled.",
        }
        if scope == self._scope:
            self.status_label.setText(state.detail or labels.get(state.state, "Navigation command in progress."))
        if state.state in {
            ApplicationCommandState.STATE_SUCCEEDED,
            ApplicationCommandState.STATE_FAILED,
            ApplicationCommandState.STATE_CANCELLED,
        }:
            self._operation_contexts.pop(state.context_id)

    @staticmethod
    def _show_assigned_name(label, name):
        label.setText(name or "Not set")
        label.setToolTip(name or "No assignment saved for this routine.")
        color = "#2e7d32" if name else "#c62828"
        label.setStyleSheet(f"color: {color}; font-weight: bold;")
