import argparse
import sys
import termios
import tty
import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from rclpy.node import Node

from typing import Optional

from bosdyn.api import geometry_pb2
from bosdyn.client import math_helpers
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn_msgs.conversions import convert
from synchros2.action_client import ActionClientWrapper
from synchros2.tf_listener_wrapper import TFListenerWrapper
from synchros2.utilities import namespace_with
import synchros2.scope as ros_scope
import synchros2.process as ros_process
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from sensor_msgs.msg import CameraInfo


from spot_msgs.action import RobotCommand
from spot_examples.simple_spot_commander import SimpleSpotCommander


class AprilTagListener(Node):
    def __init__(self):
        super().__init__('apriltag_to_pose_node')

        self.declare_parameter('tag_id', 0)
        self.declare_parameter('depth', 0.5)
        self.tag_id = self.get_parameter('tag_id').value
        self.depth = self.get_parameter('depth').value

        self.camera_info = None
        self.camera_info_sub = self.create_subscription(CameraInfo, '/camera/hand/camera_info',
                                                        self.camera_info_callback, 10)

        self.detection_sub = self.create_subscription(AprilTagDetectionArray, '/detections',
                                                      self.detection_callback, 10)
        self.pose_pub = self.create_publisher(PoseStamped, 'arm_goal', 10)

        self.get_logger().info("AprilTagToPoseNode started. Waiting for detections...")

    def camera_info_callback(self, msg: CameraInfo):
        if self.camera_info is None:
            self.camera_info = msg
            self.get_logger().info("Camera info received.")

    def detection_callback(self, msg: AprilTagDetectionArray):
        if self.camera_info is None:
            self.get_logger().warn("Waiting for camera info.")
            return

        for detection in msg.detections:
            if hasattr(detection, 'id') and detection.id == self.tag_id:
                x_pixel = detection.centre.x
                y_pixel = detection.centre.y

                x, y, z = self.pixel_to_camera_frame(x_pixel, y_pixel, self.camera_info, self.depth)

                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = msg.header.frame_id  # typically "hand_color_image_sensor"
                pose_msg.pose.position = Point(x=x, y=y, z=z)
                pose_msg.pose.orientation = Quaternion(w=1.0)

                self.pose_pub.publish(pose_msg)
                self.get_logger().info(f"Published PoseStamped for tag {self.tag_id} at ({x:.2f}, {y:.2f}, {z:.2f})")

    def pixel_to_camera_frame(self, x_pixel, y_pixel, camera_info, depth):
        fx = camera_info.k[0]
        fy = camera_info.k[4]
        cx = camera_info.k[2]
        cy = camera_info.k[5]

        x = (x_pixel - cx) * depth / fx
        y = (y_pixel - cy) * depth / fy
        z = depth
        return x, y, z


def cli() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", type=str, default=None)
    return parser


@ros_process.main(cli())
def main(args: argparse.Namespace) -> None:
    node = AprilTagListener()
    node.get_logger().info("Spinning node. Waiting for AprilTag detections...")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down due to keyboard interrupt.")
    finally:
        node.get_logger().info("Destroying node and shutting down.")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
