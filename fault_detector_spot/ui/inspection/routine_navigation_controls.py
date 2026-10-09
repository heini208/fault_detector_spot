"""Present routine navigation selections before their actions are connected."""

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QComboBox,
    QGridLayout,
    QGroupBox,
    QLabel,
    QPushButton,
)


class RoutineNavigationControls(QGroupBox):
    """Show available choices without saving or submitting navigation intent."""

    def __init__(self, parent=None):
        super().__init__("Routine Navigation", parent)
        self._scope = ("", "")
        self._map_names = ()
        self._waypoint_map = ""
        self._waypoint_names = ()

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
                button.setToolTip("This action is not available yet.")

        self.status_label = QLabel("Selections are not saved. Actions are not available yet.")
        self.status_label.setWordWrap(True)
        layout.addWidget(self.status_label, 2, 0, 1, 5)
        self.map_dropdown.currentIndexChanged.connect(self._refresh_waypoints)
        self._refresh_maps()

    def set_routine(self, object_id, routine_id, *, map_name="", waypoint_name=""):
        scope = (object_id, routine_id)
        if scope != self._scope:
            self._scope = scope
            self.map_dropdown.setCurrentIndex(0)
            self.waypoint_dropdown.setCurrentIndex(0)
        self._refresh_maps()
        self._show_assigned_name(self.map_name_label, map_name)
        self._show_assigned_name(self.waypoint_name_label, waypoint_name)

    def apply_navigation_state(self, state):
        """Use the existing map list and its map-qualified waypoint snapshot."""
        self._map_names = tuple(state.map_names)
        self._waypoint_map = state.active_map
        self._waypoint_names = tuple(state.waypoint_names)
        self._refresh_maps()

    def _refresh_maps(self):
        selected = self.map_dropdown.currentData()
        self.map_dropdown.blockSignals(True)
        self.map_dropdown.clear()
        self.map_dropdown.addItem("Select a map" if self._map_names else "No maps available", None)
        for name in self._map_names:
            self.map_dropdown.addItem(name, name)
        self.map_dropdown.setCurrentIndex(max(0, self.map_dropdown.findData(selected)))
        self.map_dropdown.blockSignals(False)
        self.map_dropdown.setEnabled(all(self._scope) and bool(self._map_names))
        self._refresh_waypoints()

    def _refresh_waypoints(self):
        selected_map = self.map_dropdown.currentData()
        selected = (
            self.waypoint_dropdown.currentData()
            if self.waypoint_dropdown.property("map_id") == selected_map else None
        )
        available = bool(selected_map) and selected_map == self._waypoint_map
        if not selected_map:
            placeholder = "Select a map first"
        elif not available:
            placeholder = "Waypoints unavailable for this map"
        elif not self._waypoint_names:
            placeholder = "No waypoints available"
        else:
            placeholder = "Select a waypoint"
        self.waypoint_dropdown.blockSignals(True)
        self.waypoint_dropdown.clear()
        self.waypoint_dropdown.addItem(placeholder, None)
        if available:
            for name in self._waypoint_names:
                self.waypoint_dropdown.addItem(name, name)
        self.waypoint_dropdown.setCurrentIndex(max(0, self.waypoint_dropdown.findData(selected)))
        self.waypoint_dropdown.setProperty("map_id", selected_map)
        self.waypoint_dropdown.blockSignals(False)
        self.waypoint_dropdown.setEnabled(
            all(self._scope) and available and bool(self._waypoint_names)
        )
        self.waypoint_dropdown.setToolTip(
            "Only the current map's waypoint list is available here."
            if selected_map and not available else ""
        )

    @staticmethod
    def _show_assigned_name(label, name):
        label.setText(name or "Not set")
        label.setToolTip(name or "No assignment saved for this routine.")
        color = "#2e7d32" if name else "#c62828"
        label.setStyleSheet(f"color: {color}; font-weight: bold;")
