import math

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QHBoxLayout, QVBoxLayout, QPushButton, QLabel, QLineEdit, QDoubleSpinBox, QComboBox, QMessageBox

from fault_detector_msgs.msg import OperationalIntent, TagElement
from fault_detector_spot.application.commanding.command_ids import OrientationModes
from fault_detector_spot.manipulation.arm_state_source import ArmStateSource, ArmStowState
from geometry_msgs.msg import Quaternion
from ..shared.control_helper import UIControlHelper
from ..shared.movement_layout import control_group


class TagNotFound(Exception):
    """Raised when the requested tag ID isn’t visible."""
    pass


class ManipulationControls(UIControlHelper):
    DEFAULT_OFFSETS = {
        "X": -0.10,
        "Y": 0.00,
        "Z": 0.05,
    }

    DEFAULT_ANGLES = {
        "Roll": 0.0,
        "Pitch": 0.0,
        "Yaw": 0.0,
    }

    def __init__(self, parent_ui: "Fault_Detector_UI"):
        self.offset_fields = {}
        self.orientation_combo = None
        self.arm_state_source = None
        self.arm_state_button = None
        self.arm_state_timer = None
        super().__init__(parent_ui)

    def init_ros_communication(self):
        if self.node is None:
            return
        self.arm_state_source = ArmStateSource(self.node)
        self.arm_state_timer = QTimer(self.ui)
        self.arm_state_timer.timeout.connect(self.refresh_arm_state)
        self.arm_state_timer.start(250)

    def make_rows(self) -> list:
        tag_row = self._make_tag_input_row()
        position = self._make_offset_row()
        orientation = self._make_orientation_offset_row()
        offsets = QHBoxLayout()
        offsets.setSpacing(20)
        offsets.addLayout(position, 1)
        offsets.addLayout(orientation, 1)
        return [
            control_group("Tag actions", tag_row),
            control_group(
                "Arm offset", offsets,
                self._make_reset_fields_and_move_relative_row(),
            ),
            control_group("Robot actions", self._make_control_row()),
        ]

    def refresh_arm_state(self):
        if self.arm_state_button is None:
            return
        source = self.arm_state_source
        state = source.stow_state() if source is not None else None
        if state is ArmStowState.STOWED:
            color = "#C62828"
            text = "● Ready Arm"
            detail = "Arm is stowed. Click to ready the arm."
            enabled = True
        elif state is ArmStowState.DEPLOYED:
            color = "#2E7D32"
            text = "● Stow Arm"
            detail = "Arm is ready. Click to stow the arm."
            enabled = True
        else:
            color = "#757575"
            text = "● Arm state unknown"
            enabled = False
            if source is None or source.last_received_at is None:
                detail = "Manipulator state is unavailable"
            elif source.is_stale():
                detail = "Manipulator state updates stopped"
            else:
                detail = "Spot reports an unknown arm stow state"
        self.arm_state_button.setText(text)
        self.arm_state_button.setStyleSheet(
            f"color: {color}; font-weight: bold;"
        )
        self.arm_state_button.setToolTip(detail)
        self.arm_state_button.setEnabled(enabled)

    def _handle_arm_state_toggle(self):
        source = self.arm_state_source
        state = source.stow_state() if source is not None else None
        if state is ArmStowState.STOWED:
            return self.ui.handle_simple_operation(
                OperationalIntent.INTENT_READY_ARM
            )
        if state is ArmStowState.DEPLOYED:
            return self.ui.handle_simple_operation(
                OperationalIntent.INTENT_STOW_ARM
            )
        self.refresh_arm_state()
        return None

    def destroy(self):
        if self.arm_state_timer is not None:
            self.arm_state_timer.stop()
            self.arm_state_timer = None
        if self.arm_state_source is not None:
            self.arm_state_source.destroy()
            self.arm_state_source = None

    def _make_tag_input_row(self) -> QHBoxLayout:
        row = QHBoxLayout()
        row.addWidget(QLabel("Tag:"))
        self.tag_dropdown = QComboBox()
        self.tag_dropdown.setMinimumWidth(100)
        self.update_tags_dropdown()

        self.submit_button = QPushButton("Move to Tag")
        self.submit_button.clicked.connect(self.handle_tag_selection)
        row.addWidget(self.tag_dropdown)
        row.addWidget(self.submit_button)

        row.addWidget(QLabel("Wait (s):"))
        self.duration_input = QDoubleSpinBox()
        self.duration_input.setRange(0.0, 60.0)
        self.duration_input.setSingleStep(1.0)
        self.duration_input.setValue(1.0)
        row.addWidget(self.duration_input)

        self.wait_time_button = QPushButton("Wait")
        self.wait_time_button.clicked.connect(
            lambda _: self.handle_full_message(
                OperationalIntent.INTENT_WAIT
            )
        )
        row.addWidget(self.wait_time_button)

        self.move_wait_button = QPushButton("Move & Wait")
        self.move_wait_button.clicked.connect(self.handle_move_and_wait)
        row.addWidget(self.move_wait_button)

        self.scan_all_button = QPushButton("Scan all in range")
        self.scan_all_button.clicked.connect(
            lambda _: self.handle_scan_all_in_range()
        )
        row.addWidget(self.scan_all_button)
        row.addStretch()
        return row

    def _make_offset_row(self) -> QHBoxLayout:
        row = QVBoxLayout()
        frame_row = QHBoxLayout()

        self.frames_dropdown = QComboBox()
        frame_row.addWidget(QLabel("Position (m) · Frame:"))
        self.update_frames_dropdown()
        frame_row.addWidget(self.frames_dropdown)
        frame_row.addStretch()
        row.addLayout(frame_row)

        self._add_offset_controls(row)

        return row

    def _make_orientation_offset_row(self):
        row = QVBoxLayout()
        mode_row = QHBoxLayout()
        self.add_orientation_dropdown(mode_row)
        mode_row.addStretch()
        row.addLayout(mode_row)
        self._add_orientation_offset_controls(row)
        return row

    def _make_reset_fields_and_move_relative_row(self):
        row = QHBoxLayout()
        reset_zero_btn = QPushButton("Set All = 0")
        reset_zero_btn.clicked.connect(self._reset_all_zero)
        row.addWidget(reset_zero_btn)

        reset_default_btn = QPushButton("Set All = Default")
        reset_default_btn.clicked.connect(self._reset_all_default)
        row.addWidget(reset_default_btn)

        self.move_by_offset_button = QPushButton("Move Arm by Offset")
        self.move_by_offset_button.clicked.connect(
            lambda _: self.handle_full_message(
                OperationalIntent.INTENT_MOVE_ARM_RELATIVE
            )
        )
        row.addStretch()
        row.addWidget(self.move_by_offset_button)

        return row

    def _reset_all_zero(self):
        """Set all offset and orientation fields to 0."""
        for axis in ("X", "Y", "Z", "Roll", "Pitch", "Yaw"):
            if axis in self.offset_fields:
                self.offset_fields[axis].setText("0.0")

    def _reset_all_default(self):
        """Restore default offsets and orientation angles."""
        for axis, value in {**self.DEFAULT_OFFSETS, **self.DEFAULT_ANGLES}.items():
            if axis in self.offset_fields:
                self.offset_fields[axis].setText(f"{value:.2f}" if axis in self.DEFAULT_OFFSETS else f"{value:.1f}")

    def _add_offset_controls(self, row: QHBoxLayout):
        self.offset_fields = {}
        for axis, dec_txt, inc_txt, dec_delta, inc_delta in [
            ("X", "backward", "forward", -0.05, +0.05),
            ("Y", "left", "right", +0.05, -0.05),
            ("Z", "down", "up", -0.05, +0.05),
        ]:
            dec = QPushButton(dec_txt)
            fld = QLineEdit()
            fld.setFixedWidth(68)
            fld.setText(f"{self.DEFAULT_OFFSETS[axis]:.2f}")
            inc = QPushButton(inc_txt)

            dec.clicked.connect(lambda _, a=axis, d=dec_delta: self._change_offset(a, d))
            inc.clicked.connect(lambda _, a=axis, d=inc_delta: self._change_offset(a, d))

            axis_row = QHBoxLayout()
            label = QLabel(axis)
            label.setFixedWidth(42)
            axis_row.addWidget(label)
            axis_row.addWidget(dec, 1)
            axis_row.addWidget(fld)
            axis_row.addWidget(inc, 1)
            row.addLayout(axis_row)

            self.offset_fields[axis] = fld

    def _add_orientation_offset_controls(self, row: QHBoxLayout):
        # Each tuple: (axis, dec_label, inc_label, dec_delta, inc_delta)
        controls = [
            ("Roll", "⟲ CCW", "⟳ CW", -5.0, +5.0),
            ("Pitch", "Down", "Up", +5.0, -5.0),
            ("Yaw", "Left", "Right", +5.0, -5.0),
        ]

        if not self.offset_fields:
            self.offset_fields = {}

        for axis, dec_txt, inc_txt, dec_delta, inc_delta in controls:
            dec = QPushButton(dec_txt)
            fld = QLineEdit()
            fld.setFixedWidth(68)
            fld.setText(f"{self.DEFAULT_ANGLES[axis]:.1f}")
            inc = QPushButton(inc_txt)

            dec.clicked.connect(lambda _, a=axis, d=dec_delta: self._change_angle(a, d))
            inc.clicked.connect(lambda _, a=axis, d=inc_delta: self._change_angle(a, d))

            axis_row = QHBoxLayout()
            label = QLabel(axis)
            label.setFixedWidth(42)
            axis_row.addWidget(label)
            axis_row.addWidget(dec, 1)
            axis_row.addWidget(fld)
            axis_row.addWidget(inc, 1)
            row.addLayout(axis_row)

            self.offset_fields[axis] = fld

    def _change_angle(self, axis: str, delta: float):
        field = self.offset_fields[axis]
        val = float(field.text()) + delta
        # Wrap around -180..180
        if val > 180.0:
            val -= 360.0
        elif val < -180.0:
            val += 360.0
        field.setText(f"{val:.1f}")

    def add_orientation_dropdown(self, row: QHBoxLayout):
        self.orientation_combo = QComboBox()
        self.orientation_combo.addItems([mode.value for mode in OrientationModes])
        row.addWidget(QLabel("Rotation (°) · Mode:"))
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

        stand_up_button = QPushButton("Stand Up")
        stand_up_button.clicked.connect(
            lambda _: self.ui.handle_simple_operation(
                OperationalIntent.INTENT_STAND_UP
            )
        )
        row.addWidget(stand_up_button)

        self.arm_state_button = QPushButton("● Arm state unknown")
        self.arm_state_button.clicked.connect(
            lambda _: self._handle_arm_state_toggle()
        )
        row.addWidget(self.arm_state_button)
        self.refresh_arm_state()

        for label, intent_id in [
            ("Gripper Toggle", OperationalIntent.INTENT_TOGGLE_GRIPPER),
            (
                "Reset State",
                OperationalIntent.INTENT_RETURN_TO_ESTOP_STATE,
            ),
        ]:
            btn = QPushButton(label)
            btn.clicked.connect(
                lambda _, value=intent_id:
                self.ui.handle_simple_operation(value)
            )
            row.addWidget(btn)
        return row

    def build_move_to_tag_intent(self) -> OperationalIntent:
        intent = OperationalIntent()
        intent.intent = OperationalIntent.INTENT_MOVE_ARM_TO_TAG
        return self.add_tag_info_to_intent(intent)

    def add_offset_to_intent(self, intent: OperationalIntent):
        ox, oy, oz, roll_deg, pitch_deg, yaw_deg = (self._get_offset(a) for a in
                                                    ("X", "Y", "Z", "Roll", "Pitch", "Yaw"))
        omode = self.orientation_combo.currentText()
        roll = math.radians(roll_deg)
        pitch = math.radians(pitch_deg)
        yaw = math.radians(yaw_deg)
        q = self._euler_to_quaternion(roll, pitch, yaw)

        frame_choice = self.frames_dropdown.currentText()
        if frame_choice == "map":
            if getattr(self.ui, "navigation_mode_label", None) and \
                    self.ui.navigation_mode_label.text() == "Navigation: OFF":
                self.show_warning(
                    "Navigation Mode Required",
                    "You are using MAP_FRAME but Navigation Mode is OFF.\n"
                    "Please enable Navigation Mode to proceed."
                )
                return None  # abort publishing command

        intent.offset.header = intent.tag.pose.header
        intent.offset.header.frame_id = frame_choice
        intent.offset.pose.position.x = ox
        intent.offset.pose.position.y = oy
        intent.offset.pose.position.z = oz
        intent.offset.pose.orientation = q

        intent.orientation_mode = omode
        return intent

    def update_frames_dropdown(self):
        self.ui.update_frames_dropdown(self.frames_dropdown)

    def update_tags_dropdown(self):
        self.ui.update_tags_dropdown(self.tag_dropdown)

    def _euler_to_quaternion(self, roll, pitch, yaw) -> Quaternion:
        """Convert Euler angles (radians) to a Quaternion message."""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q

    def add_tag_info_to_intent(self, intent: OperationalIntent):
        intent = self.add_tag_element_to_intent(intent)
        return self.add_offset_to_intent(intent)

    def _get_offset(self, axis: str) -> float:
        try:
            return float(self.offset_fields[axis].text())
        except Exception:
            return 0.0

    def add_tag_element_to_intent(
        self,
        intent: OperationalIntent,
        suppress_warnings=False,
    ):
        text = self.tag_dropdown.currentText().strip()
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
        intent.tag = tag_element
        return intent

    def handle_full_message(self, intent_id: int):
        intent = OperationalIntent()
        intent.intent = intent_id
        if (
            intent_id == OperationalIntent.INTENT_WAIT
            and self.duration_input.value() <= 0.0
        ):
            self.show_warning(
                "Invalid Wait",
                "Wait duration must be positive.",
            )
            return
        try:
            intent = self.add_tag_element_to_intent(intent, True)
        except TagNotFound:
            pass
        intent = self.add_offset_to_intent(intent)
        if not intent:
            return
        intent.duration_sec = self.duration_input.value()
        self.ui.execute_operation(intent)

    def handle_scan_all_in_range(self):
        intent = OperationalIntent()
        intent.intent = OperationalIntent.INTENT_SCAN_ALL_IN_RANGE
        intent.duration_sec = self.duration_input.value()
        intent = self.add_offset_to_intent(intent)
        if not intent:
            return

        reply = self.ask_question(
        "Confirm Scan",
        f"Scan all reachable tags for {intent.duration_sec:.1f}s?"
        )
        if reply != QMessageBox.Yes:
            self.status_label.setText("Scan canceled")
            return

        self.ui.execute_operation(intent)

    def handle_move_and_wait(self):
        try:
            intent = self.build_move_to_tag_intent()
        except TagNotFound:
            return
        intent.intent = OperationalIntent.INTENT_MOVE_ARM_TO_TAG_AND_WAIT
        intent.duration_sec = self.duration_input.value()

        pos = intent.tag.pose.pose.position
        offset = intent.offset.pose.position

        message = (
            f"Move to tag {intent.tag.id} at X={pos.x:.2f}, Y={pos.y:.2f}?\n"
            f"Offsets (X,Y,Z): {offset.x:.2f}, {offset.y:.2f}, {offset.z:.2f}\n"
            f"Orientation: {intent.orientation_mode}\n"
            f"Then wait {intent.duration_sec:.1f}s"
        )

        reply = self.ask_question("Confirm Move & Wait", message)
        if reply != QMessageBox.Yes:
            self.status_label.setText(f"Move & Wait to tag {intent.tag.id} canceled")
            return

        self.ui.execute_operation(intent)

    def handle_tag_selection(self):
        try:
            intent = self.build_move_to_tag_intent()
        except TagNotFound:
            return

        pos = intent.tag.pose.pose.position
        offset = intent.offset.pose.position

        message = (
            f"Move to tag {intent.tag.id} at X={pos.x:.2f}, Y={pos.y:.2f}?\n"
            f"Offsets (X,Y,Z): {offset.x:.2f}, {offset.y:.2f}, {offset.z:.2f}\n"
            f"Orientation: {intent.orientation_mode}"
        )
        reply = self.ask_question("Confirm Move", message)

        if reply != QMessageBox.Yes:
            self.status_label.setText(f"Move to tag {intent.tag.id} canceled")
            return

        self.ui.execute_operation(intent)
