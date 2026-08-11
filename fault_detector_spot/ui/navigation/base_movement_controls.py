import math

from PyQt5.QtWidgets import (
    QHBoxLayout, QPushButton, QLabel, QLineEdit, QComboBox, QMessageBox
)

from fault_detector_msgs.msg import OperationalIntent, TagElement
from geometry_msgs.msg import Quaternion
from ..shared.control_helper import UIControlHelper


class BaseMovementControls(UIControlHelper):
    DEFAULT_OFFSETS = {
        "X": 0.0,
        "Y": 0.0,
    }

    DEFAULT_ANGLES = {
        "Yaw": 0.0,
    }

    def __init__(self, parent_ui: "Fault_Detector_UI"):
        self.offset_fields = {}
        super().__init__(parent_ui)

    def init_ros_communication(self):
        pass

    # ---------------------- UI Construction ----------------------

    def make_rows(self):
        return [
            self._make_tag_input_row(),
            self._make_offset_row(),
            self._make_reset_and_move_row(),
            self._make_navigation_buttons_row()
        ]

    def _make_tag_input_row(self):
        row = QHBoxLayout()
        self.input_field = QLineEdit()
        self.input_field.setPlaceholderText("Enter tag ID")

        move_to_tag_btn = QPushButton("Move to Tag")
        move_to_tag_btn.clicked.connect(self.handle_move_to_tag)
        row.addWidget(self.input_field)
        row.addWidget(move_to_tag_btn)
        return row

    def _make_offset_row(self):
        row = QHBoxLayout()
        row.addWidget(QLabel("Base Offset:"))

        # Frame selection (same as ManipulationControls)
        self.frames_dropdown = QComboBox()
        row.addWidget(QLabel("Frame:"))
        self.update_frames_dropdown()
        row.addWidget(self.frames_dropdown)

        # X/Y controls
        for axis, dec_txt, inc_txt, dec_delta, inc_delta in [
            ("X", "Back", "Forward", -0.10, +0.10),
            ("Y", "Left", "Right", +0.10, -0.10),
        ]:
            dec = QPushButton(dec_txt)
            fld = QLineEdit()
            fld.setFixedWidth(50)
            fld.setText(f"{self.DEFAULT_OFFSETS[axis]:.2f}")
            inc = QPushButton(inc_txt)
            dec.clicked.connect(lambda _, a=axis, d=dec_delta: self._change_offset(a, d))
            inc.clicked.connect(lambda _, a=axis, d=inc_delta: self._change_offset(a, d))
            row.addWidget(QLabel(axis))
            row.addWidget(dec)
            row.addWidget(fld)
            row.addWidget(inc)
            self.offset_fields[axis] = fld

        # Rotation control (Yaw only)
        row.addWidget(QLabel("Yaw:"))
        dec = QPushButton("⟲ CCW")
        inc = QPushButton("⟳ CW")
        yaw_field = QLineEdit()
        yaw_field.setFixedWidth(50)
        yaw_field.setText(f"{self.DEFAULT_ANGLES['Yaw']:.1f}")
        dec.clicked.connect(lambda _, d=+5.0: self._change_angle("Yaw", d))
        inc.clicked.connect(lambda _, d=-5.0: self._change_angle("Yaw", d))
        row.addWidget(dec)
        row.addWidget(yaw_field)
        row.addWidget(inc)
        self.offset_fields["Yaw"] = yaw_field

        return row

    def _make_reset_and_move_row(self):
        row = QHBoxLayout()
        reset_zero_btn = QPushButton("Set All = 0")
        reset_zero_btn.clicked.connect(self._reset_all_zero)
        row.addWidget(reset_zero_btn)

        reset_default_btn = QPushButton("Set All = Default")
        reset_default_btn.clicked.connect(self._reset_all_default)
        row.addWidget(reset_default_btn)

        move_offset_btn = QPushButton("Move Base by Offset")
        move_offset_btn.clicked.connect(self.handle_move_base_relative)
        row.addWidget(move_offset_btn)

        return row

    def _make_navigation_buttons_row(self):
        row = QHBoxLayout()
        for label, cid in [
            ("Stand", OperationalIntent.INTENT_STAND_UP),
            (
                "Reset State",
                OperationalIntent.INTENT_RETURN_TO_ESTOP_STATE,
            ),
        ]:
            btn = QPushButton(label)
            btn.clicked.connect(
                lambda _, c=cid: self.ui.handle_simple_operation(c)
            )
            row.addWidget(btn)
        return row

    # ---------------------- Logic ----------------------

    def _change_offset(self, axis, delta):
        fld = self.offset_fields[axis]
        try:
            val = float(fld.text())
        except ValueError:
            val = 0.0
        fld.setText(f"{val + delta:.2f}")

    def _change_angle(self, axis, delta):
        fld = self.offset_fields[axis]
        try:
            val = float(fld.text())
        except ValueError:
            val = 0.0
        val += delta
        if val > 180.0:
            val -= 360.0
        elif val < -180.0:
            val += 360.0
        fld.setText(f"{val:.1f}")

    def _reset_all_zero(self):
        for fld in self.offset_fields.values():
            fld.setText("0.0")

    def _reset_all_default(self):
        for axis, val in {**self.DEFAULT_OFFSETS, **self.DEFAULT_ANGLES}.items():
            if axis in self.offset_fields:
                self.offset_fields[axis].setText(f"{val:.1f}")

    def update_frames_dropdown(self):
        self.ui.update_frames_dropdown(self.frames_dropdown)
    # ---------------------- Command Builders ----------------------

    def build_move_base_intent(self, intent_id):
        intent = OperationalIntent()
        intent.intent = intent_id

        # add tag info if available
        text = self.input_field.text().strip()
        if text.isdigit() and int(text) in self.ui.visible_tags:
            tag_element = TagElement()
            tag_element.id = int(text)
            tag_element.pose = self.ui.visible_tags[int(text)].pose
            intent.tag = tag_element

        # offset & rotation
        x = float(self.offset_fields["X"].text())
        y = float(self.offset_fields["Y"].text())
        yaw_deg = float(self.offset_fields["Yaw"].text())
        yaw = math.radians(yaw_deg)

        q = Quaternion()
        q.w = math.cos(yaw / 2.0)
        q.z = math.sin(yaw / 2.0)

        intent.offset.pose.position.x = x
        intent.offset.pose.position.y = y
        intent.offset.pose.position.z = 0.0
        intent.offset.pose.orientation = q

        intent.offset.header.frame_id = self.frames_dropdown.currentText()
        return intent

    # ---------------------- Button Handlers ----------------------

    def handle_move_base_relative(self):
        intent = self.build_move_base_intent(
            OperationalIntent.INTENT_MOVE_BASE_RELATIVE
        )
        msg = (
            f"Move base relative by X={intent.offset.pose.position.x:.2f}, "
            f"Y={intent.offset.pose.position.y:.2f}, "
            f"Yaw={math.degrees(2 * math.asin(intent.offset.pose.orientation.z)):.1f}° "
            f"in frame {intent.offset.header.frame_id}?"
        )
        if self.ask_question("Confirm Move Base Relative", msg) == QMessageBox.Yes:
            self.ui.execute_operation(intent)

    def handle_move_to_tag(self):
        text = self.input_field.text().strip()
        if not text.isdigit() or int(text) not in self.ui.visible_tags:
            self.show_warning(
                "Tag Not Found",
                "Enter the ID of a currently visible tag.",
            )
            return
        intent = self.build_move_base_intent(
            OperationalIntent.INTENT_MOVE_BASE_TO_TAG
        )
        msg = (
            f"Move base to tag {intent.tag.id} "
            f"with offset X={intent.offset.pose.position.x:.2f}, "
            f"Y={intent.offset.pose.position.y:.2f}, "
            f"Yaw={math.degrees(2 * math.asin(intent.offset.pose.orientation.z)):.1f}°?"
        )
        if self.ask_question("Confirm Move to Tag", msg) == QMessageBox.Yes:
            self.ui.execute_operation(intent)
