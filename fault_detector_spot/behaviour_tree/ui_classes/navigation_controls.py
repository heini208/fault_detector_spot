import os
import json

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QHBoxLayout, QLabel, QPushButton, QComboBox, QRadioButton, QButtonGroup, QLineEdit, \
    QSizePolicy
from ament_index_python.packages import get_package_share_directory
from fault_detector_msgs.msg import StringArray, ComplexCommand
from std_msgs.msg import String
from .UIControlHelper import UIControlHelper
from ..QOS_PROFILES import LATCHED_QOS
from ..commands.command_ids import CommandID


class NavigationControls(UIControlHelper):
    def __init__(self, parent_ui):
        self.recordings_dir = os.path.join(
            get_package_share_directory("fault_detector_spot"),
            "maps"
        )
        self.waypoint_name_field = None
        self.current_map = None
        super().__init__(parent_ui)



    def init_ros_communication(self):
        # Subscriber to current_map string topic
        self.complex_command_publisher = self.ui.complex_command_publisher

        self.node.create_subscription(
            String,
            '/active_map',
            self.current_map_callback,
            LATCHED_QOS
        )
        self.node.create_subscription(
            StringArray,
            '/map_list',
            self.map_list_callback,
            LATCHED_QOS
        )
        self.node.create_subscription(
            StringArray,
            '/waypoint_list',
            self.waypoint_list_callback,
            LATCHED_QOS
        )

    def make_rows(self):
        rows = [
            self._load_map_row(),
            self._make_create_map_row(),
            self._make_mode_row(),
            self._make_waypoint_row(),
            self._make_add_waypoint_row()
        ]
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

        row.addStretch(1)

        return row

    def current_map_callback(self, msg: String):
        self.current_map = msg.data
        # Update the label
        if self.current_map_label is not None:
            self.current_map_label.setText(f"Current Map: {self.current_map}")

    def map_list_callback(self, msg: StringArray):
        """
        Update the map dropdown with the names received from the map_list topic.
        """
        self.map_dropdown.clear()

        if not msg.names:
            self.map_dropdown.addItem("no maps saved")
            self.current_map = None
            self.current_map_label.setText("Current Map: None")
            self.mode_none.setChecked(True)
        else:
            for map_name in msg.names:
                self.map_dropdown.addItem(map_name)

    def waypoint_list_callback(self, msg: StringArray):
        """
        Update the waypoint dropdown with the names received from the waypoint_list topic.
        """
        self.waypoint_dropdown.clear()

        if not msg.names:
            self.waypoint_dropdown.addItem("no waypoints saved")
        else:
            for waypoint_name in msg.names:
                self.waypoint_dropdown.addItem(waypoint_name)

    def _make_create_map_row(self) -> QHBoxLayout:
        row = QHBoxLayout()

        # Text field for map name
        self.new_map_name_field = QLineEdit()
        self.new_map_name_field.setPlaceholderText("Enter new map name")
        row.addWidget(self.new_map_name_field)

        # Button to create empty map
        self.create_map_button = QPushButton("Initialize Empty Map")
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
        self.waypoint_name_field = QLineEdit()
        self.waypoint_name_field.setPlaceholderText("Enter waypoint name")
        self.waypoint_name_field.setFixedWidth(200)
        row.addWidget(self.waypoint_name_field)

        # Button to add current pose as waypoint
        self.add_waypoint_button = QPushButton("Add current pose as Waypoint")
        self.add_waypoint_button.clicked.connect(self.handle_add_waypoint)
        row.addWidget(self.add_waypoint_button)

        return row

    # --- Dropdown updates ---
    def update_waypoint_dropdown(self):
        """Load waypoints from JSON next to the selected map and populate dropdown with names."""
        self.waypoint_dropdown.clear()

        current_map = self.map_dropdown.currentText()
        if not current_map:
            self.waypoint_dropdown.addItem("no waypoints saved")
            return


    # --- Handlers (currently empty stubs) ---
    def handle_map_confirmed(self):
        selected_map = self.map_dropdown.currentText()
        if not selected_map or selected_map == "no maps saved":
            self.show_warning("No Map Selected", "Please select a map before confirming.")
            return

        complex_command = ComplexCommand()
        complex_command.command = self.ui.build_basic_command(CommandID.SWAP_MAP)
        complex_command.map_name = selected_map
        self.complex_command_publisher.publish(complex_command)

    def handle_mode_none(self, checked: bool):
        if checked:
            self.ui.handle_simple_command(CommandID.STOP_MAPPING)
            pass

    def handle_mode_mapping(self, checked: bool):
        if checked:
            if not self._check_current_map_selected():
                return
            self.ui.handle_simple_command(CommandID.START_SLAM)

    def handle_mode_localization(self, checked: bool):
        if checked:
            if not self._check_current_map_selected():
                return
            self.ui.handle_simple_command(CommandID.START_LOCALIZATION)

    def _check_current_map_selected(self) -> bool:
        if self.current_map is None:
            self.show_warning("No Map Selected", "Please load a map first.")
            self.mode_none.setChecked(True)  # switch to default
            return False
        return True

    def handle_add_waypoint(self):
        name = self.waypoint_name_field.text().strip()
        if not name:
            self.show_warning("Missing Name", "Please enter a waypoint name.")
            return

        if not self.current_map:
            self.show_warning("No Map Selected", "Please select a map first.")
            return

        # Check if waypoint already exists in dropdown
        existing_waypoints = [self.waypoint_dropdown.itemText(i) for i in range(self.waypoint_dropdown.count())]
        if name in existing_waypoints:
            self.show_warning("Waypoint Exists", f"A waypoint named '{name}' already exists in this map.")
            return

        complex_command = ComplexCommand()
        complex_command.command = self.ui.build_basic_command(CommandID.ADD_CURRENT_POSITION_WAYPOINT)
        complex_command.map_name = self.current_map
        complex_command.waypoint_name = name
        self.complex_command_publisher.publish(complex_command)


    def handle_move_to_waypoint(self):
        selected = self.waypoint_dropdown.currentText()
        # TODO: implement add logic
        pass

    def handle_create_empty_map(self):
        name = self.new_map_name_field.text().strip()
        if not name:
            self.show_warning("Missing Name", "Please enter a map name.")
            return

        existing_maps = [self.map_dropdown.itemText(i) for i in range(self.map_dropdown.count())]
        if name in existing_maps:
            self.show_warning("Map Name Already Taken", "Please enter a map name.")
            return

        # Send the CREATE_MAP command
        complex_command = ComplexCommand()
        complex_command.command = self.ui.build_basic_command(CommandID.CREATE_MAP)
        complex_command.map_name = name
        self.complex_command_publisher.publish(complex_command)

        # Switch mode to Mapping in the UI **without triggering the mapping handler**
        self.mode_mapping.blockSignals(True)
        self.mode_mapping.setChecked(True)
        self.mode_mapping.blockSignals(False)

    def handle_delete_map(self):
        current_map = self.map_dropdown.currentText()
        if not current_map or current_map == "no maps saved":
            self.show_warning("No Map Selected", "Please select a map to delete.")
            return

        complex_command = ComplexCommand()
        complex_command.command = self.ui.build_basic_command(CommandID.DELETE_MAP)
        complex_command.map_name = current_map
        self.complex_command_publisher.publish(complex_command)

        self.current_map = None
        self.current_map_label.setText("Current Map: None")
        self.mode_none.setChecked(True)

    def handle_delete_waypoint(self):
        selected = self.waypoint_dropdown.currentText()
        complex_command = ComplexCommand()
        complex_command.command = self.ui.build_basic_command(CommandID.DELETE_WAYPOINT)
        complex_command.map_name = self.current_map
        complex_command.waypoint_name = selected
        self.complex_command_publisher.publish(complex_command)

