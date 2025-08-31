from PyQt5.QtWidgets import QHBoxLayout, QLineEdit, QPushButton, QComboBox

from fault_detector_msgs.msg import CommandRecordControl
from fault_detector_msgs.msg import RecordingList
from fault_detector_spot.behaviour_tree.QOS_PROFILES import LATCHED_QOS
from .UIControlHelper import UIControlHelper


class RecordingControls(UIControlHelper):
    def __init__(self, parent_ui):
        super().__init__(parent_ui)

    def init_ros_communication(self):
        self.recordings_list_sub = self.node.create_subscription(
            RecordingList, "fault_detector/recordings_list", self.update_recordings_dropdown, LATCHED_QOS
        )
        self.record_control_pub = self.node.create_publisher(
            CommandRecordControl, "fault_detector/record_control", 10
        )

    def make_rows(self):
        rows = [
            self._make_recording_row()
        ]
        return rows

    def _make_recording_row(self) -> QHBoxLayout:
        row = QHBoxLayout()

        self.record_name_field = QLineEdit()
        self.record_name_field.setPlaceholderText("Recording name")
        self.record_name_field.setFixedWidth(250)
        row.addWidget(self.record_name_field)

        self.record_button = QPushButton("Start Recording")
        self.record_button.clicked.connect(self.toggle_recording)
        row.addWidget(self.record_button)

        self.recordings_dropdown = QComboBox()
        self.recordings_dropdown.addItem("No recordings available")
        row.addWidget(self.recordings_dropdown)

        self.play_button = QPushButton("Play Recording")
        self.play_button.clicked.connect(self.play_selected_recording)
        row.addWidget(self.play_button)

        self.delete_button = QPushButton("Delete")
        self.delete_button.clicked.connect(self.delete_selected_recording)
        row.addWidget(self.delete_button)

        return row

    def toggle_recording(self):
        name = self.record_name_field.text().strip()

        # Prevent starting without a name
        if self.record_button.text() == "Start Recording" and not name:
            self.show_warning("Missing name", "Please enter a recording name before starting.")
            return

        # Check for overwrite if starting
        if self.record_button.text() == "Start Recording":
            # Compare against dropdown list of existing recordings
            existing_names = [self.recordings_dropdown.itemText(i)
                              for i in range(self.recordings_dropdown.count())]
            if name in existing_names:
                if not self.ask_question(
                        "Overwrite Recording?",
                        f"A recording named '{name}' already exists.\nDo you want to overwrite it?"
                ):
                    return

        msg = CommandRecordControl()
        msg.name = name

        if self.record_button.text() == "Start Recording":
            msg.mode = "start"
            self.record_button.setText("Stop Recording")
            self.record_button.setStyleSheet("background-color: red; color: white; font-weight: bold;")
        else:
            msg.mode = "stop"
            self.record_button.setText("Start Recording")
            self.record_button.setStyleSheet("")  # Reset to default

        self.record_control_pub.publish(msg)

    def play_selected_recording(self):
        msg = CommandRecordControl()
        msg.name = self.recordings_dropdown.currentText()
        msg.mode = "play"
        self.record_control_pub.publish(msg)

    def update_recordings_dropdown(self, msg):
        self.recordings_dropdown.clear()
        self.recordings_dropdown.addItems(sorted(msg.names))

    def delete_selected_recording(self):
        current = self.recordings_dropdown.currentText()
        if current == "No recordings available":
            self.show_info("No recordings", "There are no recordings to delete.")
            return

        if not self.ask_question(
            "Delete Recording",
            f"Are you sure you want to delete '{current}'?"
        ):
            return

        msg = CommandRecordControl()
        msg.name = current
        msg.mode = "delete"
        self.record_control_pub.publish(msg)
