from PyQt5.QtWidgets import QHBoxLayout, QPushButton, QLabel, QLineEdit, QDoubleSpinBox, QComboBox, QMessageBox

from fault_detector_msgs.msg import ComplexCommand, TagElement
from fault_detector_spot.behaviour_tree.commands.command_ids import CommandID
from .UIControlHelper import UIControlHelper


class TagNotFound(Exception):
    """Raised when the requested tag ID isn’t visible."""
    pass


class ManipulationControls(UIControlHelper):
    def __init__(self, parent_ui: "Fault_Detector_UI"):
        self.offset_fields = {}
        self.orientation_combo = None
        super().__init__(parent_ui)

    def init_ros_communication(self):
        self.complex_command_publisher = self.ui.complex_command_publisher

    def make_rows(self) -> list:
        rows = [
            self._make_tag_input_row(),
            self._make_offset_row(),
            self._make_control_row()
        ]
        return rows

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

        self.wait_time_button = QPushButton("Wait")
        self.wait_time_button.clicked.connect(
            lambda _, cid=CommandID.WAIT_TIME: self.handle_full_message(cid))
        row.addWidget(self.wait_time_button)

        self.move_wait_button = QPushButton("Move & Wait")
        self.move_wait_button.clicked.connect(self.handle_move_and_wait)
        row.addWidget(self.move_wait_button)

        self.scan_all_button = QPushButton("Scan all in range")
        self.scan_all_button.clicked.connect(
            lambda _, cid=CommandID.SCAN_ALL_IN_RANGE: self.handle_scan_all_in_range(cid))
        row.addWidget(self.scan_all_button)

        self.move_base_button = QPushButton("Move Base Into Range")
        self.move_base_button.clicked.connect(self.handle_move_base_into_range)
        row.addWidget(self.move_base_button)

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
        buttons = [
            ("Stand Up", CommandID.STAND_UP),
            ("Ready Arm", CommandID.READY_ARM),
            ("Stow Arm", CommandID.STOW_ARM),
            ("Gripper Toggle", CommandID.TOGGLE_GRIPPER),
            ("Reset State", CommandID.ESTOP_STATE),
        ]
        for label, cmd_id in buttons:
            btn = QPushButton(label)
            btn.clicked.connect(lambda _, cid=cmd_id: self.ui.handle_simple_command(cid))
            row.addWidget(btn)
        return row

    def build_move_to_tag_command(self) -> ComplexCommand:
        complex_command = ComplexCommand()
        complex_command.command = self.ui.build_basic_command(CommandID.MOVE_ARM_TO_TAG)
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

    def _get_offset(self, axis: str) -> float:
        try:
            return float(self.offset_fields[axis].text())
        except Exception:
            return 0.0

    def add_tag_element_to_command(self, command: ComplexCommand, suppress_warnings=False):
        text = self.input_field.text().strip()
        if not text.isdigit():
            if not suppress_warnings:
                self.show_warning("Invalid Input", "Please enter a numeric tag ID.")
            raise TagNotFound
        tag_id = int(text)
        if tag_id not in self.ui.visible_tags:
            if not suppress_warnings:
                self.show_info("Not Found", f"Tag {tag_id} not visible.")
            raise TagNotFound
        original = self.ui.visible_tags[tag_id]

        tag_element = TagElement()
        tag_element.id = tag_id
        tag_element.pose = original.pose
        command.tag = tag_element
        return command

    def handle_full_message(self, command_id: str):
        '''
        Build the maximum possible command with available UI fields and publishes it
        '''
        complex_command = ComplexCommand()
        complex_command.command = self.ui.build_basic_command(command_id)
        try:
            complex_command = self.add_tag_element_to_command(complex_command, True)
        except TagNotFound:
            pass
        complex_command = self.add_offset_to_command(complex_command)
        complex_command.wait_time = self.duration_input.value()

        self.complex_command_publisher.publish(complex_command)
        self.status_label.setText(f"Command sent: {command_id}")

    def handle_scan_all_in_range(self, command_id: str):
        complex_command = ComplexCommand()
        complex_command.command = self.ui.build_basic_command(command_id)
        complex_command.wait_time = self.duration_input.value()
        complex_command = self.add_offset_to_command(complex_command)

        reply = self.ask_question(
        "Confirm Scan",
        f"Scan all reachable tags for {complex_command.wait_time:.1f}s?"
        )
        if reply != QMessageBox.Yes:
            self.status_label.setText("Scan canceled")
            return

        self.complex_command_publisher.publish(complex_command)
        self.status_label.setText(f"Command sent: Scan all reachable tags for {complex_command.wait_time:.1f}s")

    def handle_move_and_wait(self):
        try:
            complex_command = self.build_move_to_tag_command()
        except TagNotFound:
            return
        complex_command.command.command_id = CommandID.MOVE_ARM_TO_TAG_AND_WAIT
        complex_command.wait_time = self.duration_input.value()

        pos = complex_command.tag.pose.pose.position
        offset = complex_command.offset.pose.position

        message = (
            f"Move to tag {complex_command.tag.id} at X={pos.x:.2f}, Y={pos.y:.2f}?\n"
            f"Offsets (X,Y,Z): {offset.x:.2f}, {offset.y:.2f}, {offset.z:.2f}\n"
            f"Orientation: {complex_command.orientation_mode}\n"
            f"Then wait {complex_command.wait_time:.1f}s"
        )

        reply = self.ask_question("Confirm Move & Wait", message)
        if reply != QMessageBox.Yes:
            self.status_label.setText(f"Move & Wait to tag {complex_command.tag.id} canceled")
            return

        self.complex_command_publisher.publish(complex_command)
        self.status_label.setText(
            f"Command sent: Move & Wait to tag {complex_command.tag.id} for {complex_command.wait_time:.1f}s"
        )

    def handle_tag_selection(self):

        complex_command = self.build_move_to_tag_command()

        pos = complex_command.tag.pose.pose.position
        offset = complex_command.offset.pose.position

        message = (
            f"Move to tag {complex_command.tag.id} at X={pos.x:.2f}, Y={pos.y:.2f}?\n"
            f"Offsets (X,Y,Z): {offset.x:.2f}, {offset.y:.2f}, {offset.z:.2f}\n"
            f"Orientation: {complex_command.orientation_mode}"
        )
        reply = self.ask_question("Confirm Move", message)

        if reply != QMessageBox.Yes:
            self.status_label.setText(f"Move to tag {complex_command.tag.id} canceled")
            return

        self.complex_command_publisher.publish(complex_command)
        self.status_label.setText(f"Command sent: Move to tag {complex_command.tag.id}")

    def handle_move_base_into_range(self):
        """
        Build a ComplexCommand that requests the base to move until the tag is in arm range.
        Uses UIControlHelper dialogs for confirmation and info/warnings.
        """
        try:
            complex_command = self.build_move_to_tag_command()
        except TagNotFound:
            return

        complex_command.command.command_id = CommandID.MOVE_INTO_TAG_RANGE
        pos = complex_command.tag.pose.pose.position
        offset = complex_command.offset.pose.position

        message = (
            f"Move BASE until tag {complex_command.tag.id} is in arm range.\n"
            f"Tag at X={pos.x:.2f}, Y={pos.y:.2f}\n"
            f"Offsets (X,Y,Z): {offset.x:.2f}, {offset.y:.2f}, {offset.z:.2f}\n"
            f"Orientation: {complex_command.orientation_mode}\n"
            f"Timeout/Wait: {complex_command.wait_time:.1f}s"
        )

        reply = self.ask_question("Confirm Move Base Into Range", message)
        if reply != QMessageBox.Yes:
            if self.status_label:
                self.status_label.setText(f"Move base into range to tag {complex_command.tag.id} canceled")
            return

        self.complex_command_publisher.publish(complex_command)
        if self.status_label:
            self.status_label.setText(f"Command sent: Move base into range to tag {complex_command.tag.id}")