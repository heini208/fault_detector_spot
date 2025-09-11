#!/usr/bin/env python3
import ctypes
import struct

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from synchros2.tf_listener_wrapper import TFListenerWrapper
import tf2_sensor_msgs.tf2_sensor_msgs
from sensor_msgs_py import point_cloud2
import numpy as np
import tf2_ros

from std_msgs.msg import Header

# QoS profile for point clouds
POINT_CLOUD_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1
)


class PointCloudMerger(Node):
    def __init__(self):
        super().__init__('pointcloud_merger')

        self._init_parameters()
        self._init_subscribers()

        self.tf_listener = TFListenerWrapper(self)
        self.pub = self.create_publisher(PointCloud2, self.output_topic, POINT_CLOUD_QOS)
        self.topic_cloud_map = {topic: None for topic in self.input_topics}

        self.timer_period = 0.5  # seconds (2 Hz) – adjust as needed
        self.timer = self.create_timer(self.timer_period, self._timer_callback)
        self.min_dist = 0.02

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.topic_ready = {topic: False for topic in self.input_topics}

    def _init_parameters(self):
        self.declare_parameter('input_topics', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('output_topic', '/merged_cloud')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('remove_underscores', False)

        self.input_topics = self.get_parameter('input_topics').get_parameter_value().string_array_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.remove_underscores = self.get_parameter('remove_underscores').get_parameter_value().bool_value

    def _init_subscribers(self):
        self.subscribers = []
        for i, topic in enumerate(self.input_topics):
            sub = self.create_subscription(
                PointCloud2,
                topic,
                lambda msg, topic_name=topic: self._pointcloud_callback(msg, topic_name),
                POINT_CLOUD_QOS
            )
            self.subscribers.append(sub)
            self.get_logger().info(f'Subscribed to topic {i}: {topic}')

    def _fix_frame_id(self, frame_id: str) -> str:
        return frame_id.replace("_", " ") if self.remove_underscores else frame_id

    def _pointcloud_callback(self, msg: PointCloud2, topic_name):
        self.topic_cloud_map[topic_name] = msg
        self.topic_ready[topic_name] = True

    def _transform_cloud_msg_to_base(self, msg) -> PointCloud2:
        msg.header.frame_id = self._fix_frame_id(msg.header.frame_id)

        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, msg.header.frame_id, Time(),
                timeout=Duration(seconds=2.0)

            )
            return tf2_sensor_msgs.do_transform_cloud(msg, transform)

        except Exception as e:
            self.get_logger().warn(
                f'TF transform failed for {msg.header.frame_id}: {e}'
            )

    def _timer_callback(self):
        if not all(self.topic_ready.values()):
            return

        all_points = self._create_list_from_msgs()
        if not all_points:
            return

        merged_points = np.vstack(all_points)  # concatenate all
        unique_points = self._remove_close_points(merged_points)
        self._create_point_cloud_and_publish(unique_points)
        self.get_logger().info(
            f"Published merged cloud with {len(unique_points)} points from {len(unique_points)} inputs."
        )

    def _create_list_from_msgs(self) -> np.ndarray:
        point_list = []
        for msg in self.topic_cloud_map.values():
            if msg is not None:
                if not self.remove_underscores: # only do this if not in simulation
                    points_only_msg = point_cloud2.create_cloud_xyz32(msg.header,
                                                                  list(point_cloud2.read_points(msg, field_names=(
                                                                  "x", "y", "z"), skip_nans=True)))

                transform = self._transform_cloud_msg_to_base(points_only_msg)
                if not transform: continue
                points_structured_array = np.array(
                    list(point_cloud2.read_points(transform, field_names=("x", "y", "z"), skip_nans=True))
                )
                points_numerical = points_structured_array.view(np.float32).reshape(-1, 3)

                if points_numerical.size > 0:
                    point_list.append(points_numerical)

        return point_list

    def _create_point_cloud_and_publish(self, points: np.ndarray) -> PointCloud2:
        header = Header()
        latest_stamp = max((Time.from_msg(pc.header.stamp) for pc in self.topic_cloud_map.values()))
        header.stamp = latest_stamp.to_msg()
        header.frame_id = self.base_frame
        merged_msg = point_cloud2.create_cloud_xyz32(header, points.tolist())

        self.pub.publish(merged_msg)
        self.topic_ready = {topic: False for topic in self.input_topics}

    def _remove_close_points(self, points: np.ndarray) -> np.ndarray:
        """
        Remove points that are closer than min_dist to each other.
        Uses voxel-grid downsampling for efficiency.
        """
        if len(points) == 0:
            return points

        # Quantize coordinates into voxel grid
        voxel_indices = np.floor(points / self.min_dist).astype(np.int32)
        _, unique_idx = np.unique(voxel_indices, axis=0, return_index=True)
        return points[unique_idx]


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudMerger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
