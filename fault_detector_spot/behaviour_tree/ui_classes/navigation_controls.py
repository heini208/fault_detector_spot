import os
import json

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QHBoxLayout, QLabel, QPushButton, QComboBox, QRadioButton, QButtonGroup, QLineEdit, \
    QSizePolicy
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String
from .UIControlHelper import UIControlHelper


class NavigationControls(UIControlHelper):
    def __init__(self, parent_ui):
        self.recordings_dir = os.path.join(
            get_package_share_directory("fault_detector_spot"),
            "maps"
        )
        self.landmark_name_field = None
        self.current_map = None
        super().__init__(parent_ui)



    def init_ros_communication(self):
        # Subscriber to current_map string topic
        self.node.create_subscription(
            String,
            '/current_map',
            self.current_map_callback,
            10
        )

    def make_rows(self):
        rows = [
            self._load_map_row(),
            self._make_create_map_row(),
            self._make_mode_row(),
            self._make_landmark_row(),
            self._make_add_landmark_row()
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
        row.addWidget(self.confirm_map_button)

        self.delete_map_button = QPushButton("Delete Map")
        self.delete_map_button.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
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


    def _make_create_map_row(self) -> QHBoxLayout:
        row = QHBoxLayout()

        # Text field for map name
        self.new_map_name_field = QLineEdit()
        self.new_map_name_field.setPlaceholderText("Enter new map name")
        row.addWidget(self.new_map_name_field)

        # Button to create empty map
        self.create_map_button = QPushButton("Create Empty Map")
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

    def _make_landmark_row(self) -> QHBoxLayout:
        row = QHBoxLayout()
        row.addWidget(QLabel("Landmarks:"))

        # Dropdown for landmarks
        self.landmark_dropdown = QComboBox()
        self.update_landmark_dropdown()
        row.addWidget(self.landmark_dropdown)

        # Button to move to selected landmark
        self.move_to_landmark_button = QPushButton("Move to Landmark")
        self.move_to_landmark_button.clicked.connect(self.handle_move_to_landmark)
        row.addWidget(self.move_to_landmark_button)

        return row

    def _make_add_landmark_row(self) -> QHBoxLayout:
        row = QHBoxLayout()

        # Text field for entering landmark name
        self.landmark_name_field = QLineEdit()
        self.landmark_name_field.setPlaceholderText("Enter landmark name")
        self.landmark_name_field.setFixedWidth(200)
        row.addWidget(self.landmark_name_field)

        # Button to add current pose as landmark
        self.add_landmark_button = QPushButton("Add current pose as landmark")
        self.add_landmark_button.clicked.connect(self.handle_add_landmark)
        row.addWidget(self.add_landmark_button)

        return row

    # --- Dropdown updates ---
    def update_map_dropdown(self):
        """Populate the map dropdown with available .db files."""
        self.map_dropdown.clear()

        maps_found = False
        if os.path.isdir(self.recordings_dir):
            for f in sorted(os.listdir(self.recordings_dir)):
                if f.endswith(".db"):
                    self.map_dropdown.addItem(f[:-3])  # strip ".db"
                    maps_found = True

        if not maps_found:
            self.map_dropdown.addItem("no maps saved")

    def update_landmark_dropdown(self):
        """Load landmarks from JSON next to the selected map and populate dropdown with names."""
        self.landmark_dropdown.clear()

        current_map = self.map_dropdown.currentText()
        if not current_map:
            self.landmark_dropdown.addItem("no landmarks saved")
            return

        landmark_file = os.path.join(
            self.recordings_dir, f"{current_map}.json"
        )
        if not os.path.exists(landmark_file):
            self.landmark_dropdown.addItem("no landmarks saved")
            return

        try:
            with open(landmark_file, "r") as f:
                landmarks = json.load(f).get("landmarks", [])
        except Exception as e:
            print(f"Failed to load landmarks: {e}")
            landmarks = []

        if not landmarks:
            self.landmark_dropdown.addItem("no landmarks saved")
        else:
            for lm in landmarks:
                name = lm.get("name", f"landmark_{lm.get('id', '?')}")
                # Store full landmark as userdata on the item
                self.landmark_dropdown.addItem(name, userData=lm)

    # --- Handlers (currently empty stubs) ---
    def handle_map_confirmed(self):
        current_map = self.map_dropdown.currentText()
        if not current_map:
            self.show_warning("No Map Selected", "Please select a map before confirming.")
            return

        self.update_landmark_dropdown()

    def handle_mode_none(self, checked: bool):
        if checked:
            pass

    def handle_mode_mapping(self, checked: bool):
        if checked:
            if not self._check_current_map_selected():
                return
            # TODO: implement mapping mode logic

    def handle_mode_localization(self, checked: bool):
        if checked:
            if not self._check_current_map_selected():
                return
            # TODO: implement localization mode logic

    def _check_current_map_selected(self) -> bool:
        if self.current_map is None or self.current_map == "":
            self.show_warning("No Map Selected", "Please load a map first.")
            self.mode_none.setChecked(True)  # switch to default
            return False
        return True

    def handle_add_landmark(self):
        name = self.landmark_name_field.text().strip()
        if not name:
            self.show_warning("Missing Name", "Please enter a landmark name.")
            return

        current_map = self.map_dropdown.currentText()
        if not current_map or current_map == "no maps saved":
            self.show_warning("No Map Selected", "Please select a map first.")
            return

        # Check if landmark already exists in dropdown
        existing_landmarks = [self.landmark_dropdown.itemText(i) for i in range(self.landmark_dropdown.count())]
        if name in existing_landmarks:
            self.show_warning("Landmark Exists", f"A landmark named '{name}' already exists in this map.")
            return

        #TODO replace this with command
        json_path = os.path.join(self.recordings_dir, f"{current_map}.json")
        if os.path.exists(json_path):
            try:
                with open(json_path, "r") as f:
                    data = json.load(f)
            except Exception:
                data = {}
        else:
            data = {}

        if "landmarks" not in data:
            data["landmarks"] = []

        # Add a testing landmark
        landmark_entry = {"name": name, "pose": {}}  # empty pose for now
        data["landmarks"].append(landmark_entry)

        # Save back to JSON
        with open(json_path, "w") as f:
            json.dump(data, f, indent=4)

        self.show_info("Landmark Added", f"Added landmark '{name}' to map '{current_map}'")
        self.update_landmark_dropdown()

    def handle_move_to_landmark(self):
        selected = self.landmark_dropdown.currentText()
        # TODO: implement add landmark logic
        pass

    def handle_create_empty_map(self):
        name = self.new_map_name_field.text().strip()
        if not name:
            self.show_warning("Missing Name", "Please enter a map name.")
            return

        # Check if name already exists in dropdown
        existing_maps = [self.map_dropdown.itemText(i) for i in range(self.map_dropdown.count())]
        if name in existing_maps:
            self.show_warning("Map Exists", f"A map named '{name}' already exists.")
            return

        # TODO replace this with command
        os.makedirs(self.recordings_dir, exist_ok=True)

        db_path = os.path.join(self.recordings_dir, f"{name}.db")
        json_path = os.path.join(self.recordings_dir, f"{name}.json")

        # create empty files
        open(db_path, "a").close()
        with open(json_path, "w") as f:
            f.write("{}")  # empty JSON

        self.show_info("Map Created", f"Created empty map '{name}'")
        self.update_map_dropdown()

    def handle_delete_map(self):
        """Callback for Delete Map button (currently empty)"""
        #TODO important when deleting current publish current map none
        pass

