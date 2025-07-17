#!/usr/bin/env python3
import sys
import rclpy
import signal
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QPushButton, QLineEdit, QMessageBox, QApplication
)
from PyQt5.QtCore import QTimer
from fault_detector_msgs.msg import TagElement, TagElementArray
from std_msgs.msg import Header


class Fault_Detector_UI(QWidget):
    def __init__(self, node=None):
        super().__init__()
        self.node = node
        self.setWindowTitle("Fault Detector Spot")
        self.resize(600, 400)

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
        layout.addLayout(self._make_stow_stand_row())

    def _make_tag_input_row(self) -> QHBoxLayout:
        row = QHBoxLayout()
        self.input_field = QLineEdit()
        self.input_field.setPlaceholderText("Enter tag ID")
        self.submit_button = QPushButton("Move to Tag")
        self.submit_button.clicked.connect(self.handle_tag_selection)
        row.addWidget(self.input_field)
        row.addWidget(self.submit_button)
        return row

    def _make_stow_stand_row(self) -> QHBoxLayout:
        row = QHBoxLayout()
        # Stand Up
        self.stand_button = QPushButton("Stand Up")
        self.stand_button.clicked.connect(self.handle_stand)
        row.addWidget(self.stand_button)
        # Stow/Cancel
        self.stow_button = QPushButton("Stow Arm / Cancel")
        self.stow_button.clicked.connect(self.handle_stow)
        row.addWidget(self.stow_button)
        return row

    def init_ros_communication(self):
        # Publishers
        self.move_to_tag_publisher = self.node.create_publisher(
            TagElement, "fault_detector/commands/move_to_tag", 10
        )
        self.stow_arm_publisher = self.node.create_publisher(
            Header, "fault_detector/commands/stow_arm", 10
        )
        self.stand_up_publisher = self.node.create_publisher(
            Header, "fault_detector/commands/stand_up", 10
        )
        # Subscriber
        self.visible_tags_sub = self.node.create_subscription(
            TagElementArray,
            "fault_detector/state/visible_tags",
            self._process_visible_tags,
            10
        )
        self.status_label.setText("Status: Connected to ROS2")

    def _spin_and_refresh(self):
        """Called by QTimer: spin ROS once and update UI labels."""
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

        pos = self.visible_tags[tag_id].pose.pose.position
        reply = QMessageBox.question(
            self, "Confirm Move",
            f"Move to tag {tag_id} at X={pos.x:.2f}, Y={pos.y:.2f}?",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            self.move_to_tag_publisher.publish(self.visible_tags[tag_id])
            self.status_label.setText(f"Command sent: Move to tag {tag_id}")
        else:
            self.status_label.setText(f"Move to tag {tag_id} canceled")

    def handle_stow(self):
        """Publish a stamped Header on stow_arm topic."""
        hdr = Header()
        hdr.stamp = self.node.get_clock().now().to_msg()
        hdr.frame_id = ""
        self.stow_arm_publisher.publish(hdr)
        self.status_label.setText("Command sent: Stow/Cancel Arm")

    def handle_stand(self):
        """Publish a stamped Header on stand_up topic."""
        hdr = Header()
        hdr.stamp = self.node.get_clock().now().to_msg()
        hdr.frame_id = ""
        self.stand_up_publisher.publish(hdr)
        self.status_label.setText("Command sent: Stand Up")

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
