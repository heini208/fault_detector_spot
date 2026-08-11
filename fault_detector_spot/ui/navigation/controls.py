from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QButtonGroup,
    QComboBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QRadioButton,
    QSizePolicy,
)

from fault_detector_msgs.msg import (
    NavigationSetupIntent,
    NavigationSetupState,
    OperationalIntent,
)
from ..shared.control_helper import UIControlHelper


class NavigationControls(UIControlHelper):
    def __init__(self, parent_ui):
        self.poi_name_field = None
        self.current_map = ""
        self._cached_map_list = []
        self._cached_waypoint_list = []
        self._cached_landmark_list = []

        super().__init__(parent_ui)

    def init_ros_communication(self):
        return None

    def make_rows(self):
        rows = [
            self._load_map_row(),
            self._make_create_map_row(),
            self._make_mode_row(),
            self._make_waypoint_row(),
            self._make_add_waypoint_row(),
            self._make_landmark_row()
        ]

        self._apply_map_list()
        self._apply_waypoint_list()

        return rows

    # --- Rows ---
    def _load_map_row(self) -> QHBoxLayout:
        row = QHBoxLayout()
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(5)

        row.addWidget(QLabel("Select Map:"))

        self.map_dropdown = QComboBox()
        row.addWidget(self.map_dropdown)

        self.confirm_map_button = QPushButton("Load Map")
        self.confirm_map_button.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        self.confirm_map_button.clicked.connect(self.handle_map_confirmed)
        row.addWidget(self.confirm_map_button)

        self.delete_map_button = QPushButton("Delete Map")
        self.delete_map_button.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        self.delete_map_button.clicked.connect(self.handle_delete_map)
        row.addWidget(self.delete_map_button)

        self.current_map_label = QLabel("Current Map: None")
        self.current_map_label.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        self.current_map_label.setWordWrap(False)
        row.addWidget(self.current_map_label)
        return row

    def apply_setup_state(self, state: NavigationSetupState):
        """Render one authoritative navigation setup snapshot."""
        self.current_map = state.active_map
        self._cached_map_list = list(state.map_names)
        self._cached_waypoint_list = list(state.waypoint_names)
        self._cached_landmark_list = list(state.landmark_names)
        self.current_map_label.setText(
            f"Current Map: {self.current_map or 'None'}"
        )
        self._apply_map_list()
        self._apply_waypoint_list()
        self._apply_landmark_list()
        buttons = (
            self.mode_none,
            self.mode_mapping,
            self.mode_localization,
        )
        for button in buttons:
            button.blockSignals(True)
        self.mode_none.setChecked(
            state.mode == NavigationSetupState.MODE_NONE
        )
        self.mode_mapping.setChecked(
            state.mode == NavigationSetupState.MODE_MAPPING
        )
        self.mode_localization.setChecked(
            state.mode == NavigationSetupState.MODE_LOCALIZATION
        )
        for button in buttons:
            button.blockSignals(False)
        self.ui.set_navigation_mode(
            state.mode != NavigationSetupState.MODE_NONE
        )

    def _apply_map_list(self):
        self.map_dropdown.clear()
        if not self._cached_map_list:
            self.map_dropdown.addItem("no maps saved")
        else:
            for map_name in self._cached_map_list:
                self.map_dropdown.addItem(map_name)
            active_index = self.map_dropdown.findText(self.current_map)
            if active_index >= 0:
                self.map_dropdown.setCurrentIndex(active_index)

    def _apply_waypoint_list(self):
        self.waypoint_dropdown.clear()
        if not self._cached_waypoint_list:
            self.waypoint_dropdown.addItem("no waypoints saved")
        else:
            for wp_name in self._cached_waypoint_list:
                self.waypoint_dropdown.addItem(wp_name)

    def _apply_landmark_list(self):
        if not self._cached_landmark_list:
            self.landmark_label.setText("no landmarks saved")
            self.landmark_label.setStyleSheet("")  # default color
            self.landmark_dropdown.clear()
            self.landmark_dropdown.addItem("no landmarks saved")
            return

        # Build text string, color visible tags green/red
        text_parts = []
        for lm_name in self._cached_landmark_list:
            if lm_name.startswith("Tag_"):
                tag_id = int(lm_name.split("_")[1])
                if tag_id in self.ui.visible_tags:
                    text_parts.append(f'<span style="color: green;">{lm_name}</span>')
                else:
                    text_parts.append(f'<span style="color: red;">{lm_name}</span>')
            else:
                text_parts.append(lm_name)

        self.landmark_label.setText(", ".join(text_parts))

        # Populate the dropdown with landmarks
        self.landmark_dropdown.clear()
        for lm_name in self._cached_landmark_list:
            self.landmark_dropdown.addItem(lm_name)

    def _make_create_map_row(self) -> QHBoxLayout:
        row = QHBoxLayout()

        # Text field for map name
        self.new_map_name_field = QLineEdit()
        self.new_map_name_field.setPlaceholderText("Enter new map name")
        row.addWidget(self.new_map_name_field)

        # Button to create empty map
        self.create_map_button = QPushButton("Create Map Definition")
        self.create_map_button.clicked.connect(self.handle_create_empty_map)
        row.addWidget(self.create_map_button)

        return row

    def _make_mode_row(self) -> QHBoxLayout:
        row = QHBoxLayout()
        row.addWidget(QLabel("Mode:"))

        self.mode_none = QRadioButton("None")
        self.mode_none.setChecked(True)

        self.mode_mapping = QRadioButton("Mapping")
        self.mode_localization = QRadioButton("Localization")

        self.mode_group = QButtonGroup()
        self.mode_group.addButton(self.mode_none)
        self.mode_group.addButton(self.mode_mapping)
        self.mode_group.addButton(self.mode_localization)

        self.mode_none.toggled.connect(self.handle_mode_none)
        self.mode_mapping.toggled.connect(self.handle_mode_mapping)
        self.mode_localization.toggled.connect(self.handle_mode_localization)

        row.addWidget(self.mode_none)
        row.addWidget(self.mode_mapping)
        row.addWidget(self.mode_localization)

        return row

    def _make_waypoint_row(self) -> QHBoxLayout:
        row = QHBoxLayout()
        row.addWidget(QLabel("Waypoints:"))

        # Dropdown for waypoints
        self.waypoint_dropdown = QComboBox()
        self.update_waypoint_dropdown()
        row.addWidget(self.waypoint_dropdown)

        self.delete_waypoint_button = QPushButton("Delete Waypoint")
        self.delete_waypoint_button.clicked.connect(self.handle_delete_waypoint)
        row.addWidget(self.delete_waypoint_button)

        # Button to move to selected waypoint
        self.move_to_waypoint_button = QPushButton("Move to Waypoint")
        self.move_to_waypoint_button.clicked.connect(self.handle_move_to_waypoint)
        row.addWidget(self.move_to_waypoint_button)

        return row

    def _make_add_waypoint_row(self) -> QHBoxLayout:
        row = QHBoxLayout()

        # Text field for entering waypoint name
        self.poi_name_field = QLineEdit()
        self.poi_name_field.setPlaceholderText("Enter waypoint name or tag ID")
        self.poi_name_field.setFixedWidth(200)
        row.addWidget(self.poi_name_field)

        # Button to add current pose as waypoint
        self.add_waypoint_button = QPushButton("Add current pose as Waypoint")
        self.add_waypoint_button.clicked.connect(self.handle_add_waypoint)
        row.addWidget(self.add_waypoint_button)

        self.add_landmark_button = QPushButton("Add Tag ID as Landmark")
        self.add_landmark_button.clicked.connect(self.handle_add_landmark)
        row.addWidget(self.add_landmark_button)

        return row

    def _make_landmark_row(self) -> QHBoxLayout:
        row = QHBoxLayout()
        row.addWidget(QLabel("Landmarks:"))

        # Label showing all landmarks (rich text for coloring visible tags)
        self.landmark_label = QLabel("no landmarks saved")
        self.landmark_label.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        self.landmark_label.setTextFormat(Qt.RichText)
        row.addWidget(self.landmark_label)

        # Dropdown for deleting a landmark
        self.landmark_dropdown = QComboBox()
        row.addWidget(self.landmark_dropdown)

        # Delete landmark button
        self.delete_landmark_button = QPushButton("Delete Landmark")
        self.delete_landmark_button.clicked.connect(self.handle_delete_landmark)
        row.addWidget(self.delete_landmark_button)

        row.addStretch(1)
        return row

    # --- Dropdown updates ---
    def update_waypoint_dropdown(self):
        self.waypoint_dropdown.clear()

        current_map = self.map_dropdown.currentText()
        if not current_map:
            self.waypoint_dropdown.addItem("no waypoints saved")
            return

    def handle_map_confirmed(self):
        selected_map = self.map_dropdown.currentText()
        if not selected_map or selected_map == "no maps saved":
            self.show_warning("No Map Selected", "Please select a map before confirming.")
            return

        return self._submit_setup(
            NavigationSetupIntent.OPERATION_SELECT_MAP,
            map_name=selected_map,
        )

    def handle_mode_none(self, checked: bool):
        if checked:
            return self._submit_setup(
                NavigationSetupIntent.OPERATION_STOP_MAPPING,
                map_name=self.current_map,
            )

    def handle_mode_mapping(self, checked: bool):
        if checked:
            return self._submit_setup(
                NavigationSetupIntent.OPERATION_START_MAPPING,
                map_name=self.current_map,
            )

    def handle_mode_localization(self, checked: bool):
        if checked:
            return self._submit_setup(
                NavigationSetupIntent.OPERATION_START_LOCALIZATION,
                map_name=self.current_map,
            )

    def _check_current_map_selected(self) -> bool:
        if not self.current_map:
            self.show_warning("No Map Selected", "Please load a map first.")
            self.mode_none.setChecked(True)  # switch to default
            return False
        return True

    def handle_add_waypoint(self):
        name = self.poi_name_field.text().strip()
        if not name:
            self.show_warning("Missing Name", "Please enter a waypoint name.")
            return

        if not self._check_current_map_selected():
            return
        return self._submit_setup(
            NavigationSetupIntent.OPERATION_ADD_CURRENT_WAYPOINT,
            map_name=self.current_map,
            waypoint_name=name,
        )

    def handle_add_landmark(self):
        name = self.poi_name_field.text().strip()
        if not name:
            self.show_warning("Missing ID", "Please enter a valid tag ID")
            return

        if not name.isdigit():
            self.show_warning("Invalid Input", "Tag ID must be numeric.")
            return

        if not self._check_current_map_selected():
            return
        return self._submit_setup(
            NavigationSetupIntent.OPERATION_ADD_VISIBLE_TAG_LANDMARK,
            map_name=self.current_map,
            tag_id=int(name),
        )

    def handle_move_to_waypoint(self):
        selected = self.waypoint_dropdown.currentText()
        if (
            not self.current_map
            or not selected
            or selected == "no waypoints saved"
        ):
            self.show_warning(
                "No Waypoint Selected",
                "Select a saved waypoint before moving.",
            )
            return False
        intent = OperationalIntent()
        intent.intent = OperationalIntent.INTENT_MOVE_TO_WAYPOINT
        intent.map_name = self.current_map
        intent.waypoint_name = selected
        return self.ui.execute_operation(intent)

    def handle_create_empty_map(self):
        name = self.new_map_name_field.text().strip()
        if not name:
            self.show_warning("Missing Name", "Please enter a map name.")
            return

        existing_maps = [self.map_dropdown.itemText(i) for i in range(self.map_dropdown.count())]
        if name in existing_maps:
            self.show_warning("Map Name Already Taken", "Please enter a map name.")
            return

        return self._submit_setup(
            NavigationSetupIntent.OPERATION_CREATE_MAP_DEFINITION,
            map_name=name,
        )

    def handle_delete_map(self):
        current_map = self.map_dropdown.currentText()
        if not current_map or current_map == "no maps saved":
            self.show_warning("No Map Selected", "Please select a map to delete.")
            return

        return self._submit_setup(
            NavigationSetupIntent.OPERATION_DELETE_MAP,
            map_name=current_map,
        )

    def handle_delete_waypoint(self):
        selected = self.waypoint_dropdown.currentText()
        if not selected or selected == "no waypoints saved":
            self.show_warning(
                "No Waypoint Selected",
                "Select a waypoint to delete.",
            )
            return False
        return self._submit_setup(
            NavigationSetupIntent.OPERATION_DELETE_WAYPOINT,
            map_name=self.current_map,
            waypoint_name=selected,
        )

    def handle_delete_landmark(self):
        selected = self.landmark_dropdown.currentText()
        if not selected or selected == "no landmarks saved":
            self.show_warning("No Landmark Selected", "Please select a landmark to delete.")
            return

        return self._submit_setup(
            NavigationSetupIntent.OPERATION_DELETE_LANDMARK,
            map_name=self.current_map,
            waypoint_name=selected,
        )

    def _submit_setup(
        self,
        operation,
        map_name="",
        waypoint_name="",
        tag_id=0,
    ):
        intent = NavigationSetupIntent()
        intent.operation = operation
        intent.map_name = map_name
        intent.waypoint_name = waypoint_name
        intent.tag_id = tag_id
        return self.ui.execute_navigation_setup(intent)
