#!/usr/bin/env python3
import sys
import threading
import re
import signal

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from synchros2.tf_listener_wrapper import TFListenerWrapper
import synchros2.process as ros_process
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QLabel,
    QLineEdit, QPushButton, QMessageBox
)
from PyQt5.QtCore import QTimer

# Handle Ctrl+C in the CLI and Qt
def _handle_sigint(signum, frame):
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    raise KeyboardInterrupt

signal.signal(signal.SIGINT, _handle_sigint)

class TagToArmGoalNodeUI(Node):
    def __init__(self):
        super().__init__('tag_to_arm_goal_node')

        # State + lock for thread safety
        self._lock = threading.Lock()
        self.currently_visible_ids = set()
        self.tag_positions = {}

        # TF listener
        self.tf = TFListenerWrapper(self)

        # Publisher for arm_goal
        self._goal_pub = self.create_publisher(PoseStamped, 'arm_goal', 10)

        # Periodic TF scan
        self.create_timer(0.2, self._scan_tf)

        self.get_logger().info('Initialized. Scanning tf tree...')

    def _scan_tf(self):
        try:
            yaml = self.tf.buffer.all_frames_as_yaml()
        except Exception as e:
            self.get_logger().warn(f"Couldn’t list TF frames: {e}")
            return

        fid_re = re.compile(r"['\"]?(filtered_fiducial_(\d+))['\"]?:?")
        new_seen = set()
        new_positions = {}

        for line in yaml.splitlines():
            for m in fid_re.finditer(line):
                frame_name, tag_str = m.group(1), m.group(2)
                tag_id = int(tag_str)
                try:
                    # non-blocking check for transform
                    if not self.tf.buffer.can_transform(
                        GRAV_ALIGNED_BODY_FRAME_NAME,
                        frame_name,
                        Time(),
                        Duration(seconds=0)
                    ):
                        continue
                    tfst = self.tf.lookup_a_tform_b(
                        GRAV_ALIGNED_BODY_FRAME_NAME,
                        frame_name
                    )
                    t = tfst.transform.translation
                except Exception:
                    continue
                new_seen.add(tag_id)
                new_positions[tag_id] = (t.x, t.y, t.z)

        with self._lock:
            self.currently_visible_ids = new_seen
            self.tag_positions = new_positions

class TagToArmUI(QWidget):
    def __init__(self, tag_node: TagToArmGoalNodeUI):
        super().__init__()
        self.node = tag_node
        self.setWindowTitle('AprilFiducial Controller')
        self._build_ui()
        self._start_ui_timer()

    def _build_ui(self):
        layout = QVBoxLayout(self)

        self.visible_label = QLabel('Visible tags: []')
        layout.addWidget(self.visible_label)

        self.input_field = QLineEdit()
        self.input_field.setPlaceholderText('Enter tag ID')
        layout.addWidget(self.input_field)

        btn = QPushButton('Move Arm')
        btn.clicked.connect(self._on_submit)
        layout.addWidget(btn)

        self.status_label = QLabel('')
        layout.addWidget(self.status_label)

        self.setLayout(layout)

    def _start_ui_timer(self):
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._update_visible_tags)
        self.timer.start(100)

    def _update_visible_tags(self):
        with self.node._lock:
            tags = sorted(self.node.currently_visible_ids)
        self.visible_label.setText(f"Visible tags: {tags}")

    def _on_submit(self):
        text = self.input_field.text().strip()
        if not text.isdigit():
            QMessageBox.warning(self, 'Invalid Input', 'Please enter a numeric tag ID.')
            return
        tag_id = int(text)
        with self.node._lock:
            if tag_id not in self.node.currently_visible_ids:
                QMessageBox.information(self, 'Not Found', f'Tag {tag_id} not visible.')
                return
            pos = self.node.tag_positions.get(tag_id)
        if pos is None:
            QMessageBox.warning(self, 'Position Unknown', f'Tag {tag_id} position unavailable.')
            return
        x, y, z = pos
        reply = QMessageBox.question(
            self, 'Confirm Move',
            f'Tag {tag_id} at (x={x:.3f}, y={y:.3f}, z={z:.3f}). Move arm?',
            QMessageBox.Yes | QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            goal = PoseStamped()
            goal.header.frame_id = GRAV_ALIGNED_BODY_FRAME_NAME
            goal.pose.position.x = x
            goal.pose.position.y = y
            goal.pose.position.z = z
            self.node._goal_pub.publish(goal)
            self.status_label.setText(f'Moving to (x={x:.3f}, y={y:.3f}, z={z:.3f})')
        else:
            self.status_label.clear()
            self.input_field.clear()

    def closeEvent(self, event):
        self.node.get_logger().info('UI closed, shutting down ROS...')
        rclpy.shutdown()
        event.accept()

@ros_process.main()
def main():
    node = TagToArmGoalNodeUI()

    # Start Qt UI
    app = QApplication(sys.argv)
    ui = TagToArmUI(node)
    ui.show()

    # Spin ROS in background thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    try:
        exit_code = app.exec_()
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt, closing UI.')
        app.quit()
        exit_code = 0

    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(exit_code)

if __name__ == '__main__':
    main()
