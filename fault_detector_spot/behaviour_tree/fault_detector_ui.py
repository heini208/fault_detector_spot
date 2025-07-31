#!/usr/bin/env python3
import os
import signal
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Header, String
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QPushButton, QLineEdit, QMessageBox, QApplication, QComboBox, QDoubleSpinBox
)
from PyQt5.QtCore import QTimer, Qt
from fault_detector_msgs.msg import ComplexCommand, BasicCommand, TagElement, TagElementArray, RecordingList, CommandRecordControl
from fault_detector_spot.behaviour_tree.command_ids import CommandID
from ament_index_python.packages import get_package_share_directory


class Fault_Detector_UI(QWidget):
    def __init__(self, node: Node = None):
        super().__init__()
        self.cmd_status_sub = None
        self.buffer_sub = None
        self.visible_label = None
        self.orientation_combo = None
        self.reachable_tags_sub = None
        self.visible_tags_sub = None
        self.command_pub = None
        self.complex_command_publisher = None
        self.node = node
        self.setWindowTitle("Fault Detector Spot")
        self.resize(600, 500)

        self.status_label = QLabel("Status: Waiting for connection")
        self.buffer_label = QLabel("Buffer: []")
        self.command_status_label = QLabel("Command Status: IDLE")

        self.visible_tags = {}
        self.reachable_tags = {}
        if self.node:
            self.init_ros_communication()

        self.create_user_interface()
        # periodically spin ROS and refresh UI
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._spin_and_refresh)
        self.timer.start(10)

#------ build UI

    def create_user_interface(self):
        layout = QVBoxLayout(self)
        layout.addWidget(self.status_label)
        layout.addWidget(self.buffer_label)
        layout.addWidget(self.command_status_label)

        self.visible_label = QLabel()
        self.visible_label.setTextFormat(Qt.RichText)
        self.visible_label.setText("Visible tags: []")
        layout.addWidget(self.visible_label)

        layout.addLayout(self._make_tag_input_row())
        layout.addLayout(self._make_offset_row())
        layout.addLayout(self._make_control_row())
        layout.addLayout(self._make_recording_row())

    def _make_tag_input_row(self) -> QHBoxLayout:
        row = QHBoxLayout()
        self.input_field = QLineEdit()
        self.input_field.setPlaceholderText("Enter tag ID")

        self.submit_button = QPushButton("Move to Tag")
        self.submit_button.clicked.connect(self.handle_tag_selection)
        row.addWidget(self.input_field)
        row.addWidget(self.submit_button)

        row.addWidget(QLabel("Wait (s):"))
        self.duration_input = QDoubleSpinBox()
        self.duration_input.setRange(0.0, 60.0)
        self.duration_input.setSingleStep(1.0)
        self.duration_input.setValue(1.0)
        row.addWidget(self.duration_input)

        self.move_wait_button = QPushButton("Move & Wait")
        self.move_wait_button.clicked.connect(self.handle_move_and_wait)
        row.addWidget(self.move_wait_button)

        self.scan_all_button = QPushButton("Scan all in range")
        self.scan_all_button.clicked.connect(
            lambda _, cid=CommandID.SCAN_ALL_IN_RANGE: self.handle_scan_all_in_range(cid))
        row.addWidget(self.scan_all_button)

        return row

    def _make_offset_row(self) -> QHBoxLayout:
        row = QHBoxLayout()
        row.addWidget(QLabel("Offsets & Orientation:"))

        self._add_offset_controls(row)
        self.add_orientation_dropdown(row)
        self.move_by_offset_button = QPushButton("Move Arm by Offset")
        self.move_by_offset_button.clicked.connect(
            lambda _, cid=CommandID.MOVE_ARM_RELATIVE: self.handle_full_message(cid))
        row.addWidget(self.move_by_offset_button)

        return row

    def _add_offset_controls(self, row: QHBoxLayout):
        default_offsets = {
            "X": -0.10,
            "Y": 0.00,
            "Z": 0.05,
        }
        self.offset_fields = {}
        for axis, dec_txt, inc_txt, dec_delta, inc_delta in [
            ("X", "backward", "forward", -0.05, +0.05),
            ("Y", "left", "right", +0.05, -0.05),
            ("Z", "down", "up", -0.05, +0.05),
        ]:
            dec = QPushButton(dec_txt)
            fld = QLineEdit()
            fld.setFixedWidth(50)
            fld.setText(f"{default_offsets[axis]:.2f}")
            inc = QPushButton(inc_txt)

            dec.clicked.connect(lambda _, a=axis, d=dec_delta: self._change_offset(a, d))
            inc.clicked.connect(lambda _, a=axis, d=inc_delta: self._change_offset(a, d))

            row.addWidget(QLabel(axis))
            row.addWidget(dec)
            row.addWidget(fld)
            row.addWidget(inc)

            self.offset_fields[axis] = fld

    def add_orientation_dropdown(self, row: QHBoxLayout):
        self.orientation_combo = QComboBox()
        self.orientation_combo.addItems([
            "look_straight",
            "tag_orientation",
            "left",
            "right",
            "up",
            "down"
        ])
        row.addWidget(QLabel("Orientation:"))
        row.addWidget(self.orientation_combo)

    def _change_offset(self, axis: str, delta: float):
        fld = self.offset_fields[axis]
        try:
            val = float(fld.text())
        except ValueError:
            val = 0.0
        val += delta
        fld.setText(f"{val:.2f}")

    def _make_control_row(self) -> QHBoxLayout:
        row = QHBoxLayout()
        self.stand_button = QPushButton("Stand Up")
        self.stand_button.clicked.connect(
            lambda _, cid=CommandID.STAND_UP: self.handle_simple_command(cid)
        )
        row.addWidget(self.stand_button)

        self.ready_button = QPushButton("Ready Arm")
        self.ready_button.clicked.connect(
            lambda _, cid=CommandID.READY_ARM: self.handle_simple_command(cid)
        )
        row.addWidget(self.ready_button)

        self.stow_button = QPushButton("Stow Arm")
        self.stow_button.clicked.connect(
            lambda _, cid=CommandID.STOW_ARM: self.handle_simple_command(cid)
        )
        row.addWidget(self.stow_button)

        self.toggle_gripper_button = QPushButton("Gripper Toggle")
        self.toggle_gripper_button.clicked.connect(
            lambda _, cid=CommandID.TOGGLE_GRIPPER: self.handle_simple_command(cid)
        )
        row.addWidget(self.toggle_gripper_button)

        self.cancel_button = QPushButton("Cancel All")
        self.cancel_button.clicked.connect(
            lambda _, cid=CommandID.EMERGENCY_CANCEL: self.handle_simple_command(cid)
        )
        row.addWidget(self.cancel_button)

        return row

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

    # ---- ros communication

    def init_ros_communication(self):
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.complex_command_publisher = self.node.create_publisher(
            ComplexCommand, "fault_detector/commands/complex_command", qos
        )

        self.command_pub = self.node.create_publisher(
            BasicCommand, "fault_detector/commands/basic_command", qos)

        self.record_control_pub = self.node.create_publisher(
            CommandRecordControl, "fault_detector/record_control", 10
        )

        self.recordings_list_sub = self.node.create_subscription(
            RecordingList, "fault_detector/recordings_list", self.update_recordings_dropdown, 10
        )

        self.visible_tags_sub = self.node.create_subscription(
            TagElementArray,
            "fault_detector/state/visible_tags",
            self._process_visible_tags,
            10
        )

        self.reachable_tags_sub = self.node.create_subscription(
            TagElementArray,
            "fault_detector/state/reachable_tags",
            self._process_reachable_tags,
            10
        )

        self.buffer_sub = self.node.create_subscription(
            String,
            "fault_detector/command_buffer",
            self._process_buffer,
            10
        )
        self.cmd_status_sub = self.node.create_subscription(
            String,
            "fault_detector/command_tree_status",
            self._process_command_status,
            10
        )

        self.status_label.setText("Status: Connected to ROS2")

    def _spin_and_refresh(self):
        if self.node:
            rclpy.spin_once(self.node, timeout_sec=0.001)
        parts = []
        for tid in sorted(self.visible_tags.keys()):
            color = "green" if tid in self.reachable_tags else "red"
            parts.append(f'<span style="color:{color}">{tid}</span>')

        html = "Visible tags: [" + ", ".join(parts) + "]"
        self.visible_label.setText(html)

    def _process_visible_tags(self, msg: TagElementArray):
        self.visible_tags = {tag.id: tag for tag in msg.elements}

    def _process_reachable_tags(self, msg: TagElementArray):
        self.reachable_tags = {tag.id: tag for tag in msg.elements}

    def _process_buffer(self, msg: String):
        self.buffer_label.setText(f"Buffer: {msg.data}")

    def _process_command_status(self, msg: String):
        self.command_status_label.setText(f"Command Status: {msg.data}")

    def _get_offset(self, axis: str) -> float:
        try:
            return float(self.offset_fields[axis].text())
        except Exception:
            return 0.0

# ---- command builders

    def build_basic_command(self, command_id: str) -> BasicCommand:
        cmd = BasicCommand()
        cmd.header = Header()
        cmd.header.stamp = self.node.get_clock().now().to_msg()
        cmd.command_id = command_id
        return cmd

    def build_move_to_tag_command(self) -> ComplexCommand:
        complex_command = ComplexCommand()
        complex_command.command = self.build_basic_command(CommandID.MOVE_ARM_TO_TAG)
        complex_command = self.add_tag_info_to_command(complex_command)
        return complex_command

    def add_offset_to_command(self, command: ComplexCommand):
        ox, oy, oz = (self._get_offset(a) for a in ("X", "Y", "Z"))
        omode = self.orientation_combo.currentText()
        command.offset.header = command.tag.pose.header
        command.offset.pose.position.x = ox
        command.offset.pose.position.y = oy
        command.offset.pose.position.z = oz
        command.orientation_mode = omode
        return command

    def add_tag_info_to_command(self, command: ComplexCommand):
        command = self.add_tag_element_to_command(command)
        return self.add_offset_to_command(command)

    def add_tag_element_to_command(self, command: ComplexCommand, suppress_warnings=False):
        text = self.input_field.text().strip()
        if not text.isdigit():
            if not suppress_warnings:
                QMessageBox.warning(self, "Invalid Input", "Please enter a numeric tag ID.")
            raise TagNotFound
        tag_id = int(text)
        if tag_id not in self.visible_tags:
            if not suppress_warnings:
                QMessageBox.information(self, "Not Found", f"Tag {tag_id} not visible.")
            raise TagNotFound
        original = self.visible_tags[tag_id]

        tag_element = TagElement()
        tag_element.id = tag_id
        tag_element.pose = original.pose
        command.tag = tag_element
        return command

# ---- Button Handlers

    def handle_simple_command(self, command_id: str):
        cmd = self.build_basic_command(command_id)
        self.command_pub.publish(cmd)
        self.status_label.setText(f"Command sent: {command_id}")

    def handle_full_message(self, command_id: str):
        '''
        Build the maximum possible command with available UI fields and publishes it
        '''
        complex_command = ComplexCommand()
        complex_command.command = self.build_basic_command(command_id)
        try:
            complex_command = self.add_tag_element_to_command(complex_command, True)
        except TagNotFound:
            pass
        complex_command = self.add_offset_to_command(complex_command)

        self.complex_command_publisher.publish(complex_command)
        self.status_label.setText(f"Command sent: {command_id}")

    def handle_tag_selection(self):
        try:
            complex_command = self.build_move_to_tag_command()
        except TagNotFound:
            return

        pos = complex_command.tag.pose.pose.position
        offset = complex_command.offset.pose.position

        reply = QMessageBox.question(
            self, "Confirm Move",
            (f"Move to tag {complex_command.tag.id} at X={pos.x:.2f}, Y={pos.y:.2f}?\n"
             f"Offsets (X,Y,Z): {offset.x:.2f}, {offset.y:.2f}, {offset.z:.2f}\n"
             f"Orientation: {complex_command.orientation_mode}"),
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        )

        if reply != QMessageBox.Yes:
            self.status_label.setText(f"Move & Wait to tag {complex_command.tag.id} canceled")
            return

        self.complex_command_publisher.publish(complex_command)
        self.status_label.setText(f"Command sent: Move to tag {complex_command.tag.id}")

    def handle_move_and_wait(self):
        try:
            complex_command = self.build_move_to_tag_command()
        except TagNotFound:
            return
        complex_command.command.command_id = CommandID.MOVE_ARM_TO_TAG_AND_WAIT
        complex_command.wait_time = self.duration_input.value()

        pos = complex_command.tag.pose.pose.position
        offset = complex_command.offset.pose.position

        reply = QMessageBox.question(
            self, "Confirm Move & Wait",
            (f"Move to tag {complex_command.tag.id} at X={pos.x:.2f}, Y={pos.y:.2f}?\n"
             f"Offsets (X,Y,Z): {offset.x:.2f}, {offset.y:.2f}, {offset.z:.2f}\n"
             f"Orientation: {complex_command.orientation_mode}\n"
             f"Then wait {complex_command.wait_time:.1f}s"),
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        )
        if reply != QMessageBox.Yes:
            self.status_label.setText(f"Move & Wait to tag {complex_command.tag.id} canceled")
            return

        self.complex_command_publisher.publish(complex_command)
        self.status_label.setText(
            f"Command sent: Move & Wait to tag {complex_command.tag.id} for {complex_command.wait_time:.1f}s"
        )

    def handle_scan_all_in_range(self, command_id: str):
        complex_command = ComplexCommand()
        complex_command.command = self.build_basic_command(command_id)
        complex_command.wait_time = self.duration_input.value()
        complex_command = self.add_offset_to_command(complex_command)

        reply = QMessageBox.question(
            self, "Confirm Scan",
            f"Scan all reachable tags for {complex_command.wait_time:.1f}s?",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        )
        if reply != QMessageBox.Yes:
            self.status_label.setText("Scan canceled")
            return

        self.complex_command_publisher.publish(complex_command)
        self.status_label.setText(f"Command sent: Scan all reachable tags for {complex_command.wait_time:.1f}s")


# ---- Recording Control

    def toggle_recording(self):
        name = self.record_name_field.text().strip()

        # Prevent starting without a name
        if self.record_button.text() == "Start Recording" and not name:
            QMessageBox.warning(self, "Missing name", "Please enter a recording name before starting.")
            return

        # Check for overwrite if starting
        if self.record_button.text() == "Start Recording":
            # Compare against dropdown list of existing recordings
            existing_names = [self.recordings_dropdown.itemText(i)
                              for i in range(self.recordings_dropdown.count())]
            if name in existing_names:
                reply = QMessageBox.question(
                    self,
                    "Overwrite Recording?",
                    f"A recording named '{name}' already exists.\nDo you want to overwrite it?",
                    QMessageBox.Yes | QMessageBox.No,
                    QMessageBox.No
                )
                if reply != QMessageBox.Yes:
                    return  # Cancel if user says No

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
            QMessageBox.information(self, "No recordings", "There are no recordings to delete.")
            return

        confirm = QMessageBox.question(
            self,
            "Delete Recording",
            f"Are you sure you want to delete '{current}'?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )

        if confirm != QMessageBox.Yes:
            return

        msg = CommandRecordControl()
        msg.name = current
        msg.mode = "delete"
        self.record_control_pub.publish(msg)

    # ---- Window close event

    def closeEvent(self, event):
        self.timer.stop()
        event.accept()


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("fault_detector_ui_node")
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    app = QApplication(sys.argv)
    ui = Fault_Detector_UI(node)
    ui.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()


class TagNotFound(Exception):
    """Raised when the requested tag ID isn’t visible."""
    pass
