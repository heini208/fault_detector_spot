#!/usr/bin/env python3
import json
import os
import time
from typing import List

import rclpy
from ament_index_python.packages import get_package_share_directory
from fault_detector_msgs.msg import ComplexCommand, BasicCommand, CommandRecordControl, StringArray
from fault_detector_spot.behaviour_tree.QOS_PROFILES import COMMAND_QOS, LATCHED_QOS
from rclpy.node import Node
from rosidl_runtime_py import message_to_ordereddict
from rosidl_runtime_py import set_message_fields
from std_msgs.msg import Bool


class RecordManager(Node):
    def __init__(self):
        super().__init__('record_manager')

        # Directory for recordings
        self.recordings_dir = os.path.join(
            get_package_share_directory('fault_detector_spot'),
            'recordings'
        )
        os.makedirs(self.recordings_dir, exist_ok=True)

        # State
        self.recording = False
        self.current_name = None
        self.start_time = None
        self.temp_data: List[dict] = []
        self.delay = 0.1

        # Publishers
        self.list_pub = self.create_publisher(StringArray, 'fault_detector/recordings_list', LATCHED_QOS)
        self.playback_state_pub = self.create_publisher(Bool, 'fault_detector/playback_state', LATCHED_QOS)
        self.complex_pub = self.create_publisher(ComplexCommand, 'fault_detector/commands/complex_command', COMMAND_QOS)
        self.basic_pub = self.create_publisher(BasicCommand, 'fault_detector/commands/basic_command', COMMAND_QOS)

        # Subscribers
        self.create_subscription(CommandRecordControl, 'fault_detector/record_control', self.handle_control, 10)
        self.create_subscription(ComplexCommand, 'fault_detector/commands/complex_command', self.capture_command, 10)
        self.create_subscription(BasicCommand, 'fault_detector/commands/basic_command', self.capture_command, 10)

        # Publish initial list
        self.publish_recordings_list()

    # ----- Handle record/play/delete control -----
    def handle_control(self, msg: CommandRecordControl):
        mode = msg.mode.lower()
        name = msg.name.strip()

        if mode == 'start':
            self.start_recording(name)
        elif mode == 'stop':
            self.stop_recording()
        elif mode == 'play':
            self.play_recording(name)
        elif mode == 'delete':
            self.delete_recording(name)

    # ----- Recording -----
    def start_recording(self, name: str):
        if not name:
            self.get_logger().warn("Recording name is empty, ignoring.")
            return
        self.delete_recording(name)
        self.recording = True
        self.current_name = name
        self.temp_data.clear()
        self.start_time = time.time()
        self.get_logger().info(f"Started recording: {name}")

    def stop_recording(self):
        if not self.recording:
            return
        file_path = os.path.join(self.recordings_dir, f"{self.current_name}.json")
        with open(file_path, 'w') as f:
            json.dump(self.temp_data, f, indent=2)
        self.get_logger().info(f"Saved recording: {file_path}")
        self.recording = False
        self.current_name = None
        self.publish_recordings_list()

    def capture_command(self, msg):
        if not self.recording:
            return

        # Convert message to dict
        msg_dict = {
            "topic": 'complex' if isinstance(msg, ComplexCommand) else 'basic',
            "timestamp": round(time.time() - self.start_time, 2),
            "data": self.serialize_ros_message(msg)
        }
        self.temp_data.append(msg_dict)

    # ----- Playback -----
    def play_recording(self, name: str):
        file_path = os.path.join(self.recordings_dir, f"{name}.json")
        if not os.path.exists(file_path):
            self.get_logger().warn(f"Recording not found: {name}")
            return

        self.get_logger().info(f"Playing back: {name}")
        self.playback_state_pub.publish(Bool(data=True))

        with open(file_path, 'r') as f:
            data = json.load(f)

        for entry in data:
            if entry["topic"] == "complex":
                msg = ComplexCommand()
                self.deserialize_ros_message(entry["data"], msg)
                msg.command.header.stamp = self.get_clock().now().to_msg()
                self.complex_pub.publish(msg)
            else:
                msg = BasicCommand()
                self.deserialize_ros_message(entry["data"], msg)
                msg.header.stamp = self.get_clock().now().to_msg()
                self.basic_pub.publish(msg)
            # small delay
            time.sleep(self.delay)

        self.playback_state_pub.publish(Bool(data=False))
        self.get_logger().info("Playback finished.")

    # ----- Delete -----
    def delete_recording(self, name: str):
        file_path = os.path.join(self.recordings_dir, f"{name}.json")
        if os.path.exists(file_path):
            os.remove(file_path)
            self.get_logger().info(f"Deleted recording: {name}")
            self.publish_recordings_list()

    # ----- Recordings list -----
    def publish_recordings_list(self):
        files = [f[:-5] for f in os.listdir(self.recordings_dir) if f.endswith(".json")]
        self.list_pub.publish(StringArray(names=sorted(files)))

    # ----- Serialization helpers -----
    def serialize_ros_message(self, msg):
        """Convert ROS message to plain dict."""
        return message_to_ordereddict(msg)

    def deserialize_ros_message(self, data_dict, msg):
        """Fill ROS message from dict."""
        set_message_fields(msg, data_dict)


def main(args=None):
    rclpy.init(args=args)
    node = RecordManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
