from PyQt5.QtCore import QObject, Qt, pyqtSignal
from std_msgs.msg import Bool
from PyQt5.QtWidgets import (
    QHBoxLayout, QLineEdit, QPushButton, QComboBox, QSpinBox, QWidget,
)

from fault_detector_msgs.msg import CommandRecordControl
from fault_detector_msgs.msg import StringArray
from fault_detector_spot.shared.ros.qos_profiles import LATCHED_QOS
from ..shared.control_helper import UIControlHelper


class _RecordingSignals(QObject):
    state_changed = pyqtSignal(bool)
    paused_changed = pyqtSignal(bool)
    finishing_changed = pyqtSignal(bool)
    list_changed = pyqtSignal(object)


class RecordingControls(UIControlHelper):
    def __init__(self, parent_ui):
        self._recording = None
        self._recording_paused = None
        self._recording_finishing = False
        self._signals = _RecordingSignals(parent_ui)
        self._signals.state_changed.connect(self.update_recording_state, Qt.QueuedConnection)
        self._signals.paused_changed.connect(self.update_recording_paused, Qt.QueuedConnection)
        self._signals.finishing_changed.connect(self.update_recording_finishing, Qt.QueuedConnection)
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
        self.recording_paused_sub = self.node.create_subscription(
            Bool, "fault_detector/recording_paused",
            lambda message: self._signals.paused_changed.emit(message.data), LATCHED_QOS,
        )
        self.recording_finishing_sub = self.node.create_subscription(
            Bool, "fault_detector/recording_finishing",
            lambda message: self._signals.finishing_changed.emit(message.data), LATCHED_QOS,
        )

    def make_rows(self):
        rows = [
            self._make_recording_row(),
            self._make_recording_options_row(),
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

    def _make_recording_options_row(self) -> QHBoxLayout:
        self.recording_options = QWidget()
        controls = QHBoxLayout(self.recording_options)
        controls.setContentsMargins(0, 0, 0, 0)

        self.skip_button = QPushButton("Start skip block")
        self.skip_button.setCheckable(True)
        self.skip_button.setToolTip(
            "Start a skip block; press again to end it. During playback, if a "
            "command fails after its retries, skip the rest of this block. "
            "Successful commands are not undone. Skip blocks cannot be nested."
        )
        self.skip_button.toggled.connect(self._update_skip_button)
        controls.addWidget(self.skip_button)

        self.retry_button = QPushButton()
        self.retry_button.setCheckable(True)
        self.retry_button.setToolTip(
            "Allow the selected number of additional attempts for the next "
            "recorded command, only when retrying is supported and safe. "
            "Press again to clear the selection."
        )
        self.retry_button.toggled.connect(self._update_retry_button)
        controls.addWidget(self.retry_button)
        self.retry_count = QSpinBox()
        self.retry_count.setRange(0, 100)
        self.retry_count.setValue(1)
        self.retry_count.setAccessibleName("Additional retries for next command")
        self.retry_count.setToolTip(
            "Additional attempts after the first attempt: 0 means no retries, "
            "1 allows up to 2 attempts."
        )
        self.retry_count.valueChanged.connect(self._update_retry_button)
        controls.addWidget(self.retry_count)
        self._update_retry_button()

        self.pause_recording_button = QPushButton("Don't record")
        self.pause_recording_button.setCheckable(True)
        self.pause_recording_button.setToolTip(
            "Exclude subsequent commands from the recording JSON. Commands "
            "still execute and diagnostic logging continues. Press again to "
            "resume including commands in the recording."
        )
        self.pause_recording_button.clicked.connect(self.toggle_recording_pause)
        self._refresh_pause_button()
        controls.addWidget(self.pause_recording_button)
        controls.addStretch()

        # Skip and retry remain presentation-only until playback supports them.
        self.recording_options.setEnabled(bool(self._recording))
        row = QHBoxLayout()
        row.addWidget(self.recording_options)
        return row

    @staticmethod
    def _set_option_active(button, active):
        button.setStyleSheet(
            "background-color: red; color: white; font-weight: bold;"
            if active else ""
        )

    def _update_skip_button(self, active):
        self.skip_button.setText("Stop skip block" if active else "Start skip block")
        self._set_option_active(self.skip_button, active)

    def _update_retry_button(self, *_):
        count = self.retry_count.value()
        noun = "retry" if count == 1 else "retries"
        self.retry_button.setText(f"Allow {count} {noun} for next command")
        self._set_option_active(self.retry_button, self.retry_button.isChecked())

    def _update_pause_button(self, active):
        self.pause_recording_button.setText(
            "Recording paused" if active else "Don't record"
        )
        self._set_option_active(self.pause_recording_button, active)

    def _refresh_pause_button(self):
        active = bool(self._recording and self._recording_paused)
        self.pause_recording_button.setChecked(active)
        self._update_pause_button(active)
        self.pause_recording_button.setEnabled(
            bool(self._recording) and self._recording_paused is not None
        )

    def update_recording_paused(self, paused):
        self._recording_paused = paused
        self._refresh_pause_button()

    def toggle_recording_pause(self):
        # Render only confirmed state, not the button's optimistic toggle.
        self._refresh_pause_button()
        if not self._recording or self._recording_finishing or self._recording_paused is None:
            return
        msg = CommandRecordControl()
        msg.mode = "resume" if self._recording_paused else "pause"
        self.record_control_pub.publish(msg)

    def toggle_recording(self):
        if self._recording is None or (self._recording and self._recording_finishing):
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
        if recording != self._recording:
            self.skip_button.setChecked(False)
            self.retry_button.setChecked(False)
        self._recording = recording
        self._refresh_pause_button()
        self._refresh_recording_button()
        self.record_button.setStyleSheet(
            "background-color: red; color: white; font-weight: bold;" if recording else ""
        )
        self.record_name_field.setEnabled(not recording)

    def update_recording_finishing(self, finishing):
        self._recording_finishing = finishing
        self._refresh_recording_button()

    def _refresh_recording_button(self):
        finishing = bool(self._recording and self._recording_finishing)
        self.recording_options.setEnabled(bool(self._recording) and not finishing)
        self.record_button.setEnabled(self._recording is not None and not finishing)
        self.record_button.setText(
            "Finishing recording…" if finishing else
            "Stop Recording" if self._recording else
            "Start Recording" if self._recording is not None else
            "Waiting for recording state"
        )
        self.record_button.setToolTip(
            "Waiting for pending command outcomes before saving. "
            "New commands are no longer included."
            if finishing else ""
        )

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
