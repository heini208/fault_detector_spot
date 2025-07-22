#!/usr/bin/env python3
import signal
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QPushButton, QLineEdit, QMessageBox, QApplication, QComboBox
)
from PyQt5.QtCore import QTimer
from fault_detector_msgs.msg import TagElement, TagElementArray, BasicCommand


class Fault_Detector_UI(QWidget):
    def __init__(self, node: Node = None):
        super().__init__()
        self.node = node
        self.setWindowTitle("Fault Detector Spot")
        self.resize(600, 500)

        self.visible_tags = {}
        self.create_user_interface()

        if self.node:
            self.init_ros_communication()

        # periodically spin ROS and refresh UI
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._spin_and_refresh)
        self.timer.start(100)

    def create_user_interface(self):
        layout = QVBoxLayout(self)

        self.status_label = QLabel("Status: Waiting for connection")
        layout.addWidget(self.status_label)

        self.visible_label = QLabel("Visible tags: []")
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
        self.stand_button.clicked.connect(self.handle_stand)
        row.addWidget(self.stand_button)

        self.ready_button = QPushButton("Ready Arm")
        self.ready_button.clicked.connect(self.handle_ready)
        row.addWidget(self.ready_button)

        self.stow_button = QPushButton("Stow Arm / Cancel")
        self.stow_button.clicked.connect(self.handle_stow)
        row.addWidget(self.stow_button)

        return row

    def init_ros_communication(self):
        self.move_to_tag_publisher = self.node.create_publisher(
            TagElement, "fault_detector/commands/move_to_tag", 10
        )

        self.command_pub = self.node.create_publisher(
            BasicCommand, "fault_detector/commands/basic_command", 10)

        self.visible_tags_sub = self.node.create_subscription(
            TagElementArray,
            "fault_detector/state/visible_tags",
            self._process_visible_tags,
            10
        )
        self.status_label.setText("Status: Connected to ROS2")

    def _spin_and_refresh(self):
        if self.node:
            rclpy.spin_once(self.node, timeout_sec=0.01)
        self.visible_label.setText(f"Visible tags: {sorted(self.visible_tags.keys())}")

    def _process_visible_tags(self, msg: TagElementArray):
        self.visible_tags = {tag.id: tag for tag in msg.elements}

    def _get_offset(self, axis: str) -> float:
        try:
            return float(self.offset_fields[axis].text())
        except Exception:
            return 0.0

    def handle_tag_selection(self):
        text = self.input_field.text().strip()
        if not text.isdigit():
            QMessageBox.warning(self, "Invalid Input", "Please enter a numeric tag ID.")
            return
        tag_id = int(text)
        if tag_id not in self.visible_tags:
            QMessageBox.information(self, "Not Found", f"Tag {tag_id} not visible.")
            return

        original = self.visible_tags[tag_id]
        pos = original.pose.pose.position
        ox, oy, oz = (self._get_offset(a) for a in ("X","Y","Z"))
        omode = self.orientation_combo.currentText()
        reply = QMessageBox.question(
            self, "Confirm Move",
            (f"Move to tag {tag_id} at X={pos.x:.2f}, Y={pos.y:.2f}?\n"
             f"Offsets (X,Y,Z): {ox:.2f}, {oy:.2f}, {oz:.2f}\n"
             f"Orientation: {omode}"),
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            msg = TagElement()
            msg.id = tag_id
            msg.pose = original.pose
            msg.offset.header = original.pose.header
            msg.offset.pose.position.x = ox
            msg.offset.pose.position.y = oy
            msg.offset.pose.position.z = oz
            msg.orientation_mode = omode
            self.move_to_tag_publisher.publish(msg)
            self.status_label.setText(f"Command sent: Move to tag {tag_id}")
        else:
            self.status_label.setText(f"Move to tag {tag_id} canceled")

    def handle_stand(self):
        cmd = BasicCommand()
        cmd.header = Header()
        cmd.header.stamp = self.node.get_clock().now().to_msg()
        cmd.command_id = "stand_up"
        self.command_pub.publish(cmd)
        self.status_label.setText("Command sent: Stand Up")

    def handle_ready(self):
        cmd = BasicCommand()
        cmd.header = Header()
        cmd.header.stamp = self.node.get_clock().now().to_msg()
        cmd.command_id = "ready_arm"
        self.command_pub.publish(cmd)
        self.status_label.setText("Command sent: Ready Arm")

    def handle_stow(self):
        cmd = BasicCommand()
        cmd.header = Header()
        cmd.header.stamp = self.node.get_clock().now().to_msg()
        cmd.command_id = "stow_arm"
        self.command_pub.publish(cmd)
        self.status_label.setText("Command sent: Stow Arm / Cancel")

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
