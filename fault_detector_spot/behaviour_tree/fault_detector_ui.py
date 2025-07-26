#!/usr/bin/env python3
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
from fault_detector_msgs.msg import TagElement, TagElementArray, BasicCommand
from fault_detector_spot.behaviour_tree.command_ids import CommandID


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
        self.move_to_tag_publisher = None
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
        return row

    def _make_offset_row(self) -> QHBoxLayout:
        row = QHBoxLayout()
        row.addWidget(QLabel("Offsets & Orientation:"))

        self._add_offset_controls(row)
        self.add_orientation_dropdown(row)
        return row

    def _add_offset_controls(self, row: QHBoxLayout):
        self.offset_fields = {}
        for axis, dec_txt, inc_txt, dec_delta, inc_delta in [
            ("X", "backward", "forward", -0.05, +0.05),
            ("Y", "left", "right", +0.05, -0.05),
            ("Z", "down", "up", -0.05, +0.05),
        ]:
            dec = QPushButton(dec_txt)
            fld = QLineEdit()
            fld.setFixedWidth(50)
            fld.setPlaceholderText("0.0")
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

        self.cancel_button = QPushButton("Cancel All")
        self.cancel_button.clicked.connect(
            lambda _, cid=CommandID.EMERGENCY_CANCEL: self.handle_simple_command(cid)
        )
        row.addWidget(self.cancel_button)

        return row

    def init_ros_communication(self):
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.move_to_tag_publisher = self.node.create_publisher(
            TagElement, "fault_detector/commands/move_to_tag", qos
        )

        self.command_pub = self.node.create_publisher(
            BasicCommand, "fault_detector/commands/basic_command", qos)

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

    def handle_tag_selection(self):
        tag_element = self.build_basic_tag_element()
        if tag_element is None:
            return
        pos = tag_element.pose.pose.position
        offset = tag_element.offset.pose.position

        reply = QMessageBox.question(
            self, "Confirm Move",
            (f"Move to tag {tag_element.id} at X={pos.x:.2f}, Y={pos.y:.2f}?\n"
             f"Offsets (X,Y,Z): {offset.x:.2f}, {offset.y:.2f}, {offset.z:.2f}\n"
             f"Orientation: {tag_element.orientation_mode}"),
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        )

        if reply != QMessageBox.Yes:
            self.status_label.setText(f"Move & Wait to tag {tag_element.id} canceled")
            return

        self.move_to_tag_publisher.publish(tag_element)
        self.status_label.setText(f"Command sent: Move to tag {tag_element.id}")

    def handle_move_and_wait(self):
        tag_element = self.build_basic_tag_element()
        if tag_element is None:
            return
        pos = tag_element.pose.pose.position
        offset = tag_element.offset.pose.position
        tag_element.duration = self.duration_input.value()
        reply = QMessageBox.question(
            self, "Confirm Move & Wait",
            (f"Move to tag {tag_element.id} at X={pos.x:.2f}, Y={pos.y:.2f}?\n"
             f"Offsets (X,Y,Z): {offset.x:.2f}, {offset.y:.2f}, {offset.z:.2f}\n"
             f"Orientation: {tag_element.orientation_mode}\n"
             f"Then wait {tag_element.duration:.1f}s"),
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        )
        if reply != QMessageBox.Yes:
            self.status_label.setText(f"Move & Wait to tag {tag_element.id} canceled")
            return

        self.move_to_tag_publisher.publish(tag_element)
        self.status_label.setText(
            f"Command sent: Move & Wait to tag {tag_element.id} for {tag_element.duration:.1f}s"
        )

    def build_basic_tag_element(self):
        text = self.input_field.text().strip()
        if not text.isdigit():
            QMessageBox.warning(self, "Invalid Input", "Please enter a numeric tag ID.")
            return
        tag_id = int(text)
        if tag_id not in self.visible_tags:
            QMessageBox.information(self, "Not Found", f"Tag {tag_id} not visible.")
            return

        original = self.visible_tags[tag_id]
        ox, oy, oz = (self._get_offset(a) for a in ("X", "Y", "Z"))
        omode = self.orientation_combo.currentText()
        msg = TagElement()
        msg.id = tag_id
        msg.pose = original.pose
        msg.offset.header = original.pose.header
        msg.offset.pose.position.x = ox
        msg.offset.pose.position.y = oy
        msg.offset.pose.position.z = oz
        msg.orientation_mode = omode

        return msg

    def handle_simple_command(self, command_id: str):
        cmd = BasicCommand()
        cmd.header = Header()
        cmd.header.stamp = self.node.get_clock().now().to_msg()
        cmd.command_id = command_id
        self.command_pub.publish(cmd)
        self.status_label.setText(f"Command sent: {command_id}")

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
