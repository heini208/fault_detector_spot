#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, CameraInfo
from fault_detector_spot.behaviour_tree.QOS_PROFILES import POINT_CLOUD_QOS
from rclpy.parameter import Parameter

import message_filters


class PointCloudRepublisher(Node):
    def __init__(self):
        super().__init__('pointcloud_republisher')

        self._load_parameters()
        self._setup_publishers()
        self._setup_subscribers()

        self.get_logger().info(
            f"Republishing {len(self.input_topics)} topics to {self.output_topic} with sync"
        )

    def _load_parameters(self):
        self.declare_parameter('input_topics', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('output_topic', '/merged_cloud')
        self.declare_parameter('remove_underscores', False)

        self.input_topics = self.get_parameter(
            'input_topics').get_parameter_value().string_array_value
        self.output_topic = self.get_parameter(
            'output_topic').get_parameter_value().string_value
        self.remove_underscores = self.get_parameter(
            'remove_underscores').get_parameter_value().bool_value

    def _setup_publishers(self):
        self.pub = self.create_publisher(PointCloud2, self.output_topic, POINT_CLOUD_QOS)
        self.cam_info_pub = self.create_publisher(CameraInfo, '/merged_camera_info', 10)

    def _setup_subscribers(self):
        # Create subscribers via message_filters
        self.subs = [
            message_filters.Subscriber(self, PointCloud2, topic, qos_profile=POINT_CLOUD_QOS)
            for topic in self.input_topics
        ]

        # Synchronizer: waits until all topics publish before firing callback
        self.ts = message_filters.ApproximateTimeSynchronizer(
            self.subs,
            queue_size=10,
            slop=0.1  # seconds of tolerance for time mismatch
        )
        self.ts.registerCallback(self.synced_callback)

    def _fix_frame_id(self, frame_id: str) -> str:
        return frame_id.replace("_", " ") if self.remove_underscores else frame_id

    def synced_callback(self, *msgs):
        """
        This callback is called once we have one message from each input topic.
        """
        for msg in msgs:
            msg.header.frame_id = self._fix_frame_id(msg.header.frame_id)

            # Republish point cloud
            self.pub.publish(msg)

            # Publish minimal CameraInfo
            cam_info = CameraInfo()
            cam_info.header.stamp = msg.header.stamp
            cam_info.header.frame_id = msg.header.frame_id
            self.cam_info_pub.publish(cam_info)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Republisher...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
