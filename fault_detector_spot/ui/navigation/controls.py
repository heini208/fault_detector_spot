from PyQt5.QtWidgets import (
    QButtonGroup,
    QComboBox,
    QCheckBox,
    QHBoxLayout,
    QVBoxLayout,
    QGroupBox,
    QWidget,
    QListWidget,
    QToolButton,
    QMenu,
    QLabel,
    QLineEdit,
    QPushButton,
    QRadioButton,
)

from fault_detector_msgs.msg import (
    NavigationSetupIntent,
    NavigationSetupState,
    OperationalIntent,
)
from ..shared.control_helper import UIControlHelper
from ..shared.collapsible_section import CollapsibleSection
from fault_detector_spot.shared.persistence.basic_settings import startup_settings


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

    def add_rows(self, layout):
        for row in self.rows:
            layout.addLayout(row)

    def make_rows(self):
        workspace = QVBoxLayout()
        workspace.setSpacing(14)
        map_group = QGroupBox("Map && mode")
        map_layout = QVBoxLayout(map_group)
        map_layout.setSpacing(10)
        map_layout.addLayout(self._load_map_row())
        map_layout.addWidget(self.current_map_label)
        map_layout.addLayout(self._make_mode_row())
        create_panel = QWidget()
        create_layout = QVBoxLayout(create_panel)
        create_layout.setContentsMargins(0, 0, 0, 0)
        create_layout.addLayout(self._make_create_map_row())
        map_layout.addWidget(CollapsibleSection("Create map", create_panel))
        workspace.addWidget(map_group)

        panels = QHBoxLayout()
        panels.setSpacing(14)
        waypoint_group = QGroupBox("Waypoints")
        waypoint_layout = QVBoxLayout(waypoint_group)
        waypoint_layout.addLayout(self._make_waypoint_row())
        waypoint_layout.addLayout(self._make_add_waypoint_row())
        self.waypoint_hint = QLabel("Load a map to use waypoints.")
        self.waypoint_hint.setWordWrap(True)
        waypoint_layout.addWidget(self.waypoint_hint)
        panels.addWidget(waypoint_group, 1)

        landmark_group = QGroupBox("Landmarks")
        landmark_layout = QVBoxLayout(landmark_group)
        landmark_layout.addLayout(self._make_landmark_row())
        landmark_layout.addLayout(self._make_add_landmark_row())
        self.landmark_hint = QLabel("Load a map to use landmarks.")
        self.landmark_hint.setWordWrap(True)
        landmark_layout.addWidget(self.landmark_hint)
        panels.addWidget(landmark_group, 1)
        workspace.addLayout(panels)
        workspace.addStretch(1)

        self._apply_map_list()
        self._apply_waypoint_list()
        self._apply_landmark_list()
        self.update_tags_dropdown()
        self._refresh_availability()
        return [workspace]

    def _load_map_row(self):
        row = QHBoxLayout()
        row.setSpacing(8)
        row.addWidget(QLabel("Map:"))
        self.map_dropdown = QComboBox()
        self.map_dropdown.setMinimumWidth(220)
        row.addWidget(self.map_dropdown, 1)
        self.confirm_map_button = QPushButton("Load")
        self.confirm_map_button.clicked.connect(self.handle_map_confirmed)
        row.addWidget(self.confirm_map_button)
        self.delete_map_button = QPushButton("Delete map")
        self.delete_map_button.clicked.connect(self.handle_delete_map)
        self.delete_map_button.hide()
        more = QToolButton()
        more.setText("⋯")
        more.setAccessibleName("Map actions")
        more.setToolTip("Map actions")
        menu = QMenu(more)
        delete = menu.addAction("Delete map…")
        delete.triggered.connect(self.delete_map_button.click)
        menu.aboutToShow.connect(lambda: delete.setEnabled(self.delete_map_button.isEnabled()))
        more.setMenu(menu)
        more.setPopupMode(QToolButton.InstantPopup)
        row.addWidget(more)
        row.addStretch()
        self.current_map_label = QLabel("Current map: None")
        return row

    def apply_setup_state(self, state: NavigationSetupState):
        """Render one authoritative navigation setup snapshot."""
        self.current_map = state.active_map
        self._cached_map_list = list(state.map_names)
        self._cached_waypoint_list = list(state.waypoint_names)
        self._cached_landmark_list = list(state.landmark_names)
        self.current_map_label.setText(
            f"Current map: {self.current_map or 'None'}"
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
        self._refresh_availability()

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

    @staticmethod
    def _populate_list(widget, names):
        selected = widget.currentItem().text() if widget.currentItem() else None
        widget.blockSignals(True)
        widget.clear()
        widget.addItems(names)
        if names:
            widget.setCurrentRow(names.index(selected) if selected in names else 0)
        widget.blockSignals(False)

    def _apply_waypoint_list(self):
        self._populate_list(self.waypoint_list, self._cached_waypoint_list)

    def _apply_landmark_list(self):
        self._populate_list(self.landmark_list, self._cached_landmark_list)

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

        self.mode_none = QRadioButton("Off")
        self.mode_none.setChecked(True)

        self.mode_mapping = QRadioButton("Mapping")
        self.mode_mapping.setToolTip(
            "Existing maps require active localization with verified lidar-map "
            "matches before switching to Mapping. Only new maps can start "
            "mapping directly from Off."
        )
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
        self.mapping_rviz = QCheckBox("Show RViz")
        self.mapping_rviz.setChecked(startup_settings().get('mapping_rviz', True))
        self.mapping_rviz.setToolTip(
            "Show or hide RViz without restarting mapping or localization. "
            "Also applies to the next launch."
        )
        self.mapping_rviz.toggled.connect(self.handle_mapping_visualization)
        row.addWidget(self.mapping_rviz)
        row.addStretch()

        return row

    def _make_waypoint_row(self):
        layout = QVBoxLayout()
        self.waypoint_list = QListWidget()
        self.waypoint_list.setMinimumHeight(130)
        self.waypoint_list.setMaximumHeight(210)
        self.waypoint_list.setStyleSheet("QListWidget::item { padding: 6px; }")
        self.waypoint_list.currentRowChanged.connect(self._refresh_availability)
        layout.addWidget(self.waypoint_list)
        actions = QHBoxLayout()
        self.move_to_waypoint_button = QPushButton("Move to waypoint")
        self.move_to_waypoint_button.setMinimumHeight(34)
        self.move_to_waypoint_button.setStyleSheet(
            "QPushButton:enabled { background: palette(highlight); color: palette(highlighted-text); "
            "font-weight: bold; border: none; border-radius: 4px; padding: 6px 12px; }"
        )
        self.move_to_waypoint_button.clicked.connect(self.handle_move_to_waypoint)
        actions.addWidget(self.move_to_waypoint_button, 1)
        self.delete_waypoint_button = QPushButton("Delete")
        self.delete_waypoint_button.clicked.connect(self.handle_delete_waypoint)
        actions.addWidget(self.delete_waypoint_button)
        layout.addLayout(actions)
        return layout

    def _make_add_waypoint_row(self):
        row = QHBoxLayout()
        self.poi_name_field = QLineEdit()
        self.poi_name_field.setPlaceholderText("Waypoint name")
        row.addWidget(self.poi_name_field, 1)
        self.add_waypoint_button = QPushButton("Add current pose")
        self.add_waypoint_button.clicked.connect(self.handle_add_waypoint)
        row.addWidget(self.add_waypoint_button)
        return row

    def _make_landmark_row(self):
        layout = QVBoxLayout()
        self.landmark_list = QListWidget()
        self.landmark_list.setMinimumHeight(130)
        self.landmark_list.setMaximumHeight(210)
        self.landmark_list.setStyleSheet("QListWidget::item { padding: 6px; }")
        self.landmark_list.currentRowChanged.connect(self._refresh_availability)
        layout.addWidget(self.landmark_list)
        actions = QHBoxLayout()
        self.landmark_label = QLabel("Saved localization references")
        actions.addWidget(self.landmark_label, 1)
        self.delete_landmark_button = QPushButton("Delete")
        self.delete_landmark_button.setMinimumHeight(34)
        self.delete_landmark_button.clicked.connect(self.handle_delete_landmark)
        actions.addWidget(self.delete_landmark_button)
        layout.addLayout(actions)
        return layout

    def _make_add_landmark_row(self):
        row = QHBoxLayout()
        row.addWidget(QLabel("Visible tag:"))
        self.landmark_tag_dropdown = QComboBox()
        self.landmark_tag_dropdown.setMinimumWidth(90)
        self.landmark_tag_dropdown.setToolTip("Select a visible tag to save as a landmark.")
        row.addWidget(self.landmark_tag_dropdown, 1)
        self.add_landmark_button = QPushButton("Add tag")
        self.add_landmark_button.clicked.connect(self.handle_add_landmark)
        row.addWidget(self.add_landmark_button)
        return row

    def _refresh_availability(self, *_args):
        loaded = bool(self.current_map)
        has_maps = bool(self._cached_map_list)
        self.confirm_map_button.setEnabled(has_maps)
        self.delete_map_button.setEnabled(has_maps)
        self.mode_mapping.setEnabled(loaded)
        self.mode_localization.setEnabled(loaded)
        has_waypoint = loaded and self.waypoint_list.currentItem() is not None
        self.move_to_waypoint_button.setEnabled(has_waypoint)
        self.delete_waypoint_button.setEnabled(has_waypoint)
        self.add_waypoint_button.setEnabled(loaded)
        self.delete_landmark_button.setEnabled(loaded and self.landmark_list.currentItem() is not None)
        self.add_landmark_button.setEnabled(loaded and bool(self.ui.visible_tags))
        self.waypoint_hint.setText(
            "Load a map to use waypoints." if not loaded else
            "No waypoints saved. Add the current robot pose." if not self._cached_waypoint_list else
            "Select a waypoint, then move to it."
        )
        self.landmark_hint.setText(
            "Load a map to use landmarks." if not loaded else
            "No visible tags. A visible tag is required to add a landmark." if not self.ui.visible_tags else
            "Save a visible tag as a localization reference."
        )
        for index in range(self.landmark_list.count()):
            item = self.landmark_list.item(index)
            tag = item.text().removeprefix("Tag_")
            item.setToolTip("Tag visible" if tag.isdigit() and int(tag) in self.ui.visible_tags
                            else "Tag not currently visible")

    def update_tags_dropdown(self):
        self.ui.update_tags_dropdown(self.landmark_tag_dropdown)
        self._refresh_availability()

    def handle_map_confirmed(self):
        selected_map = self.map_dropdown.currentText()
        if not selected_map or selected_map == "no maps saved":
            self.show_warning("No Map Selected", "Please select a map before confirming.")
            return

        return self._submit_setup(
            NavigationSetupIntent.OPERATION_SELECT_MAP,
            map_name=selected_map,
        )

    def handle_mapping_visualization(self, checked: bool):
        if self.mode_mapping.isChecked() or self.mode_localization.isChecked():
            return self._submit_setup(
                NavigationSetupIntent.OPERATION_SET_MAPPING_VISUALIZATION,
                map_name=self.current_map,
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
        selected = self.landmark_tag_dropdown.currentText()
        if not selected.isdigit() or int(selected) not in self.ui.visible_tags:
            self.show_warning("No Visible Tag Selected", "Please select a currently visible tag.")
            return

        if not self._check_current_map_selected():
            return
        return self._submit_setup(
            NavigationSetupIntent.OPERATION_ADD_VISIBLE_TAG_LANDMARK,
            map_name=self.current_map,
            tag_id=int(selected),
        )

    def handle_move_to_waypoint(self):
        selected = self.waypoint_list.currentItem().text() if self.waypoint_list.currentItem() else ""
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
        selected = self.waypoint_list.currentItem().text() if self.waypoint_list.currentItem() else ""
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
        selected = self.landmark_list.currentItem().text() if self.landmark_list.currentItem() else ""
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
        intent.mapping_rviz = self.mapping_rviz.isChecked()
        intent.waypoint_name = waypoint_name
        intent.tag_id = tag_id
        return self.ui.execute_navigation_setup(intent)
