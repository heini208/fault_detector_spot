import sys
import rclpy
import threading
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QLabel,
    QLineEdit, QPushButton, QMessageBox
)
from PyQt5.QtCore import QTimer


class TagTracker(Node):
    """
    ROS node responsible for subscribing to AprilTag detections
    and maintaining current visibility and center positions.
    """
    def __init__(self):
        super().__init__('tag_tracker')
        self._lock = threading.Lock()
        self.currently_visible_ids = set()
        self.detection_centers = {}
        self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self._on_detections,
            10
        )

    def _on_detections(self, msg: AprilTagDetectionArray):
        new_seen = {det.id for det in msg.detections}
        new_centers = {det.id: (det.centre.x, det.centre.y)
                       for det in msg.detections}
        with self._lock:
            self.currently_visible_ids = new_seen
            self.detection_centers = new_centers

    def get_visible_tags(self):
        with self._lock:
            return set(self.currently_visible_ids)

    def get_center(self, tag_id: int):
        with self._lock:
            return self.detection_centers.get(tag_id)


class InputValidator:
    """
    Helper for validating and parsing user input.
    """
    @staticmethod
    def parse_tag(text: str):
        if not text.isdigit():
            raise ValueError('Please enter a number.')
        tag_id = int(text)
        if not (1 <= tag_id <= 586):
            raise ValueError('Tag ID must be between 1 and 586.')
        return tag_id


class TagToArmUI(QWidget):
    def __init__(self, tag_tracker: TagTracker):
        super().__init__()
        self.node = tag_tracker
        self.setWindowTitle('AprilTag Controller')
        self._build_ui()
        self._start_ros_spin()

    def _build_ui(self):
        layout = QVBoxLayout(self)

        self.visible_label = QLabel('Visible tags: []')
        layout.addWidget(self.visible_label)

        self.input_field = QLineEdit()
        self.input_field.setPlaceholderText('Enter tag ID (1-586)')
        layout.addWidget(self.input_field)

        self.submit_btn = QPushButton('Submit')
        self.submit_btn.clicked.connect(self._on_submit)
        layout.addWidget(self.submit_btn)

        self.status_label = QLabel('')
        layout.addWidget(self.status_label)

        self.setLayout(layout)

    def _start_ros_spin(self):
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._update_visible_tags)
        self.timer.timeout.connect(lambda: rclpy.spin_once(self.node, timeout_sec=0))
        self.timer.start(50)

    def _update_visible_tags(self):
        tags = sorted(self.node.get_visible_tags())
        self.visible_label.setText(f"Visible tags: {tags}")

    def _on_submit(self):
        text = self.input_field.text().strip()
        try:
            tag_id = InputValidator.parse_tag(text)
        except ValueError as e:
            QMessageBox.warning(self, 'Invalid Input', str(e))
            return

        visible = self.node.get_visible_tags()
        if tag_id not in visible:
            QMessageBox.information(
                self, 'Not Found', f'Tag {tag_id} not visible.'
            )
            return

        center = self.node.get_center(tag_id)
        x, y = center
        reply = QMessageBox.question(
            self, 'Confirm Move',
            f'Tag {tag_id} at (x={x:.2f}, y={y:.2f}). Move arm?',
            QMessageBox.Yes | QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            self._start_move(x, y)
        else:
            self.status_label.clear()

    def _start_move(self, x: float, y: float):
        # TODO: publish PoseStamped to arm controller
        self.status_label.setText(f'Moving to (x={x:.2f}, y={y:.2f})')


def main():
    rclpy.init()
    tag_tracker = TagTracker()
    app = QApplication(sys.argv)
    ui = TagToArmUI(tag_tracker)
    ui.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
