#!/usr/bin/env python3
import signal
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QPushButton, QLineEdit, QMessageBox, QApplication
)
from PyQt5.QtCore import QTimer
from fault_detector_msgs.msg import TagElement, TagElementArray, BasicCommand


class Fault_Detector_UI(QWidget):
    def __init__(self, node: Node = None):
        super().__init__()
        self.node = node
        self.setWindowTitle("Fault Detector Spot")
        self.resize(600, 450)

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
        row.addWidget(QLabel("Offsets:"))

        # for each axis, make [-] [field] [+]
        self.offset_fields = {}
        for axis in ("X", "Y", "Z"):
            dec = QPushButton("–")
            fld = QLineEdit()
            fld.setFixedWidth(50)
            fld.setPlaceholderText("0.0")
            inc = QPushButton("+")
            # closures to capture axis
            dec.clicked.connect(lambda _, a=axis: self._change_offset(a, -0.1))
            inc.clicked.connect(lambda _, a=axis: self._change_offset(a, +0.1))
            row.addWidget(QLabel(axis))
            row.addWidget(dec)
            row.addWidget(fld)
            row.addWidget(inc)
            self.offset_fields[axis] = fld

        return row

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
        reply = QMessageBox.question(
            self, "Confirm Move",
            f"Move to tag {tag_id} at X={pos.x:.2f}, Y={pos.y:.2f}?\n"
            f"With offsets X/Y/Z = "
            f"{self._get_offset('X'):.2f}, {self._get_offset('Y'):.2f}, {self._get_offset('Z'):.2f}",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            # build new TagElement message
            msg = TagElement()
            msg.id = tag_id
            msg.pose = original.pose
            # stamp & frame for offset same as pose
            msg.offset.header = original.pose.header
            off = msg.offset.pose.position
            off.x, off.y, off.z = self._get_offset("X"), self._get_offset("Y"), self._get_offset("Z")
            # publish
            self.move_to_tag_publisher.publish(msg)
            self.status_label.setText(f"Command sent: Move to tag {tag_id}")
        else:
            self.status_label.setText(f"Move to tag {tag_id} canceled")

    def _get_offset(self, axis: str) -> float:
        fld = self.offset_fields[axis]
        try:
            return float(fld.text())
        except Exception:
            return 0.0

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

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
