#!/usr/bin/env python3
import sys
import rclpy
import threading
import signal  # Import the signal module
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel,
                             QPushButton, QLineEdit, QMessageBox, QApplication)
from PyQt5.QtCore import QTimer
from fault_detector_msgs.msg import TagElement, TagElementArray


class Fault_Detector_UI(QWidget):
    # ... (class content is unchanged) ...
    def __init__(self, node=None):
        super().__init__()
        self.node = node
        self.setWindowTitle("Fault Detector Spot")
        self.resize(600, 400)

        self.create_user_interface()

        if self.node:
            self.init_ros_communication()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh_user_interface)
        self.timer.start(100)

        # Change to a dictionary to store tag poses
        self.visible_tags = {}

    def create_user_interface(self):
        layout = QVBoxLayout(self)

        self.status_label = QLabel("Status: Waiting for connection")
        layout.addWidget(self.status_label)

        self.visible_label = QLabel('Visible tags: []')
        layout.addWidget(self.visible_label)

        input_layout = self.create_tag_input_section()
        layout.addLayout(input_layout)

    def create_tag_input_section(self):
        input_layout = QHBoxLayout()

        self.input_field = QLineEdit()
        self.input_field.setPlaceholderText('Enter tag ID')

        self.submit_button = QPushButton('Move to Tag')
        self.submit_button.clicked.connect(self.handle_tag_selection)

        input_layout.addWidget(self.input_field)
        input_layout.addWidget(self.submit_button)

        return input_layout

    def init_ros_communication(self):
        self.init_publishers()
        self.init_subscribers()

        self.status_label.setText("Status: Connected to ROS2")

    def init_publishers(self):
        self.move_to_tag_publisher = self.node.create_publisher(
            TagElement, "fault_detector/commands/move_to_tag", 10)

    def init_subscribers(self):
        self.visible_tags_subscriber = self.node.create_subscription(
            TagElementArray, "fault_detector/state/visible_tags", self.process_visible_tags_message, 10)

    def process_visible_tags_message(self, msg):
        try:
            self.visible_tags = {tag.id: tag for tag in msg.elements}
        except Exception as e:
            self.status_label.setText(f"Error parsing tags message: {str(e)}")

    def refresh_user_interface(self):
        self.visible_label.setText(f"Visible tags: {sorted(self.visible_tags.keys())}")

    def handle_tag_selection(self):
        if not self.node:
            self.status_label.setText("Error: Not connected to ROS2")
            return

        text = self.input_field.text().strip()
        if not text.isdigit():
            QMessageBox.warning(self, 'Invalid Input', 'Please enter a numeric tag ID.')
            return

        tag_id = int(text)
        if tag_id not in self.visible_tags:
            QMessageBox.information(self, 'Not Found', f'Tag {tag_id} not visible.')
            return

        reply = self.get_user_confirmation(tag_id)

        if reply == QMessageBox.Yes:
            self.send_move_to_tag_command(tag_id)
            self.status_label.setText(f"Command sent: Move to tag {tag_id}")
        else:
            self.status_label.setText(f"Move to tag {tag_id} cancelled.")

    def get_tag_position(self, tag_id):
        tag_element = self.visible_tags[tag_id]
        return tag_element.pose.pose.position

    def get_user_confirmation(self, tag_id):
        pos = self.get_tag_position(tag_id)
        return QMessageBox.question(self, 'Confirm Move',
                                     f'Move to tag {tag_id} at position X={pos.x:.2f}, Y={pos.y:.2f}?',
                                     QMessageBox.Yes | QMessageBox.No, QMessageBox.No)


    def send_move_to_tag_command(self, tag_id):
        if not hasattr(self, 'move_to_tag_publisher'):
            return

        try:
            msg = self.visible_tags[tag_id]
            self.move_to_tag_publisher.publish(msg)
        except Exception as e:
            self.status_label.setText(f"Error sending command: {str(e)}")

    def closeEvent(self, event):
        self.timer.stop()
        event.accept()


def main(args=None):
    """Initializes the ROS2 node and the PyQt5 application."""
    rclpy.init(args=args)
    node = rclpy.create_node('fault_detector_ui_node')
    app = QApplication(sys.argv)

    # Add this line to handle Ctrl+C
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    ui = Fault_Detector_UI(node)

    # Use a QTimer to periodically spin the ROS node
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.01))
    timer.start(50)

    ui.show()

    try:
        exit_code = app.exec_()
        sys.exit(exit_code)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()