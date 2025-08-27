#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, CameraInfo
from fault_detector_spot.behaviour_tree.QOS_PROFILES import POINT_CLOUD_QOS
from rclpy.parameter import Parameter

class PointCloudMerger(Node):
    def __init__(self):
        super().__init__('pointcloud_republisher')

        self._load_parameters()
        self._setup_publishers()
        self._setup_subscribers()

        self.get_logger().info(
            f"Republishing {len(self.input_topics)} topics to {self.output_topic}"
        )

    def _load_parameters(self):
        self.declare_parameter('input_topics', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('output_topic', '/merged_cloud')
        self.declare_parameter('remove_underscores', False)

        self.input_topics = self.get_parameter('input_topics').get_parameter_value().string_array_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.remove_underscores = self.get_parameter('remove_underscores').get_parameter_value().bool_value

    def _setup_publishers(self):
        self.pub = self.create_publisher(PointCloud2, self.output_topic, POINT_CLOUD_QOS)
        self.cam_info_pub = self.create_publisher(CameraInfo, '/merged_camera_info', 10)

    def _setup_subscribers(self):
        for topic in self.input_topics:
            self.create_subscription(
                PointCloud2,
                topic,
                lambda msg, t=topic: self.pc_callback(msg, t),
                POINT_CLOUD_QOS
            )

    def _fix_frame_id(self, frame_id: str) -> str:
        return frame_id.replace("_", " ") if self.remove_underscores else frame_id

    def pc_callback(self, msg: PointCloud2, topic_name: str):
        # Fix frame_id needed due to weebot error
        msg.header.frame_id = self._fix_frame_id(msg.header.frame_id)

        # Republish the point cloud
        self.pub.publish(msg)

        # Publish corresponding CameraInfo
        cam_info = CameraInfo()
        cam_info.header.stamp = msg.header.stamp
        cam_info.header.frame_id = msg.header.frame_id
        self.cam_info_pub.publish(cam_info)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
