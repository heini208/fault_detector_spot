#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, CameraInfo
from sensor_msgs_py import point_cloud2
import tf2_ros
import tf2_sensor_msgs.tf2_sensor_msgs
from fault_detector_spot.behaviour_tree.QOS_PROFILES import POINT_CLOUD_QOS
from rclpy.parameter import Parameter


class PointCloudMerger(Node):
    def __init__(self):
        super().__init__('pointcloud_merger')

        self._load_parameters()
        self._setup_tf()
        self._setup_publisher()
        self._initialize_storage()
        self._setup_subscribers()

        self.get_logger().info(
            f"Merging {len(self.input_topics)} topics into {self.output_topic} in frame {self.base_frame}"
        )

    def _load_parameters(self):
        # Declare parameters
        self.declare_parameter('input_topics', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('output_topic', '/merged_cloud')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('remove_underscores', False)

        # Get parameter values
        self.input_topics = self.get_parameter('input_topics').get_parameter_value().string_array_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.remove_underscores = self.get_parameter('remove_underscores').get_parameter_value().bool_value

    def _setup_tf(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def _setup_publisher(self):
        self.pub = self.create_publisher(PointCloud2, self.output_topic, POINT_CLOUD_QOS)
        self.cam_info_pub = self.create_publisher(CameraInfo, '/merged_camera_info', 10)

    def _initialize_storage(self):
        # Track latest cloud per topic
        self.latest_clouds = {topic: None for topic in self.input_topics}
        self.fields = None
        self.header = None

    def _setup_subscribers(self):
        for topic in self.input_topics:
            # Use a lambda to pass the topic name to the callback
            self.create_subscription(
                PointCloud2,
                topic,
                lambda msg, t=topic: self.pc_callback(msg, t),
                POINT_CLOUD_QOS
            )

    def _remove_underscore_if_needed(self, frame_id: str) -> str:
        if self.remove_underscores:
            return frame_id.replace("_", " ")
        return frame_id

    def _transform_to_base(self, msg: PointCloud2) -> PointCloud2 | None:
        try:
            fixed_frame_id = self._remove_underscore_if_needed(msg.header.frame_id)
            msg.header.frame_id = fixed_frame_id

            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                fixed_frame_id,
                rclpy.time.Time()
            )
            return tf2_sensor_msgs.do_transform_cloud(msg, transform)
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"TF lookup failed for {msg.header.frame_id}: {e}")
            return None

    def pc_callback(self, msg: PointCloud2, topic_name: str):
        cloud_transformed = self._transform_to_base(msg)
        if cloud_transformed is None:
            return

        # Store the latest cloud for this topic
        self.latest_clouds[topic_name] = cloud_transformed

        # Only merge and publish if all topics have a cloud
        if all(self.latest_clouds.values()):
            merged_msg = self._merge_clouds()
            if merged_msg:
                self.pub.publish(merged_msg)

                cam_info = CameraInfo()
                cam_info.header.stamp = merged_msg.header.stamp
                cam_info.header.frame_id = self.base_frame
                self.cam_info_pub.publish(cam_info)

    def _merge_clouds(self) -> PointCloud2 | None:
        accumulated_points = []

        for cloud in self.latest_clouds.values():
            if cloud is None:
                continue
            else:
                self.fields = cloud.fields
                self.header = cloud.header

            for point in point_cloud2.read_points(cloud, skip_nans=True):
                accumulated_points.append(point)

        if not accumulated_points or self.header is None or self.fields is None:
            return None

        return point_cloud2.create_cloud(self.header, self.fields, accumulated_points)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
