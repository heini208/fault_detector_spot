#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, CameraInfo
from sensor_msgs_py import point_cloud2
import tf2_ros
import tf2_sensor_msgs.tf2_sensor_msgs
from rclpy.parameter import Parameter
from geometry_msgs.msg import TransformStamped
from fault_detector_spot.behaviour_tree.QOS_PROFILES import POINT_CLOUD_QOS

class SynchronizedPointCloudMerger(Node):
    def __init__(self):
        super().__init__('synchronized_pointcloud_merger')

        self.declare_parameter('input_topics', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('output_topic', '/merged_cloud')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('remove_underscores', False)

        self.input_topics = self.get_parameter('input_topics').get_parameter_value().string_array_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.remove_underscores = self.get_parameter('remove_underscores').get_parameter_value().bool_value

        # TF setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publisher
        self.pub = self.create_publisher(PointCloud2, self.output_topic, POINT_CLOUD_QOS)
        self.cam_info_pub = self.create_publisher(CameraInfo, '/merged_camera_info', 10)

        # Storage for latest clouds and flags
        self.latest_clouds = {topic: None for topic in self.input_topics}
        self.received_flags = {topic: False for topic in self.input_topics}

        # Subscribe
        for topic in self.input_topics:
            self.create_subscription(
                PointCloud2,
                topic,
                lambda msg, t=topic: self.pc_callback(msg, t),
                POINT_CLOUD_QOS
            )

        self.fields = None
        self.header = None

        self.get_logger().info(
            f"Synchronized merging of {len(self.input_topics)} topics into {self.output_topic}"
        )

    def _remove_underscore_if_needed(self, frame_id: str) -> str:
        return frame_id.replace("_", " ") if self.remove_underscores else frame_id

    def _transform_cloud(self, cloud: PointCloud2) -> PointCloud2 | None:
        """Transform a single cloud to the base frame using latest available transform."""
        try:
            fixed_frame_id = self._remove_underscore_if_needed(cloud.header.frame_id)
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                self.base_frame,
                fixed_frame_id,
                rclpy.time.Time(),  # latest transform
            )
            return tf2_sensor_msgs.do_transform_cloud(cloud, transform)
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"TF lookup failed for {fixed_frame_id}: {e}")
            return None

    def pc_callback(self, msg: PointCloud2, topic_name: str):
        # Store raw cloud without transforming yet
        self.latest_clouds[topic_name] = msg
        self.received_flags[topic_name] = True

        # Only merge when all sensors have delivered new data
        if all(self.received_flags.values()):
            # Transform all clouds to base frame at the same time
            transformed_clouds = []
            for topic, cloud in self.latest_clouds.items():
                transformed = self._transform_cloud(cloud)
                if transformed:
                    transformed_clouds.append(transformed)
                else:
                    self.get_logger().warn(f"Skipping cloud from {topic} due to transform failure")

            merged_msg = self._merge_clouds(transformed_clouds)
            if merged_msg:
                merged_msg.header.frame_id = self.base_frame
                self.pub.publish(merged_msg)

                cam_info = CameraInfo()
                cam_info.header.stamp = merged_msg.header.stamp
                cam_info.header.frame_id = self.base_frame
                self.cam_info_pub.publish(cam_info)

            # Reset flags for next round
            self.received_flags = {topic: False for topic in self.input_topics}

    def _merge_clouds(self, clouds: list[PointCloud2]) -> PointCloud2 | None:
        accumulated_points = []

        for cloud in clouds:
            if cloud is None:
                continue
            self.fields = cloud.fields
            self.header = cloud.header

            for point in point_cloud2.read_points(cloud, skip_nans=True):
                accumulated_points.append(point)

        if not accumulated_points or self.header is None or self.fields is None:
            return None

        return point_cloud2.create_cloud(self.header, self.fields, accumulated_points)


def main(args=None):
    rclpy.init(args=args)
    node = SynchronizedPointCloudMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down SynchronizedPointCloudMerger...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
