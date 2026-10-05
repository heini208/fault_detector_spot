from PyQt5.QtCore import QObject, Qt, pyqtSignal
from std_msgs.msg import Bool
from PyQt5.QtWidgets import QHBoxLayout, QLineEdit, QPushButton, QComboBox

from fault_detector_msgs.msg import CommandRecordControl
from fault_detector_msgs.msg import StringArray
from fault_detector_spot.shared.ros.qos_profiles import LATCHED_QOS
from ..shared.control_helper import UIControlHelper


class _RecordingSignals(QObject):
    state_changed = pyqtSignal(bool)
    list_changed = pyqtSignal(object)


class RecordingControls(UIControlHelper):
    def __init__(self, parent_ui):
        self._recording = None
        self._signals = _RecordingSignals(parent_ui)
        self._signals.state_changed.connect(self.update_recording_state, Qt.QueuedConnection)
        self._signals.list_changed.connect(self.update_recordings_dropdown, Qt.QueuedConnection)
        super().__init__(parent_ui)

    def init_ros_communication(self):
        self.recordings_list_sub = self.node.create_subscription(
            StringArray, "fault_detector/recordings_list", self._signals.list_changed.emit, LATCHED_QOS
        )
        self.recording_state_sub = self.node.create_subscription(
            Bool, "fault_detector/recording_state",
            lambda message: self._signals.state_changed.emit(message.data), LATCHED_QOS,
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

        self.record_button = QPushButton("Waiting for recording state")
        self.record_button.setEnabled(False)
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
        if self._recording is None:
            return
        name = self.record_name_field.text().strip()

        # Prevent starting without a name
        if not self._recording and not name:
            self.show_warning("Missing name", "Please enter a recording name before starting.")
            return

        # Check for overwrite if starting
        if not self._recording:
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

        msg.mode = "stop" if self._recording else "start"
        self.record_control_pub.publish(msg)

    def update_recording_state(self, recording):
        self._recording = recording
        self.record_button.setEnabled(True)
        self.record_button.setText("Stop Recording" if recording else "Start Recording")
        self.record_button.setStyleSheet(
            "background-color: red; color: white; font-weight: bold;" if recording else ""
        )
        self.record_name_field.setEnabled(not recording)

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
