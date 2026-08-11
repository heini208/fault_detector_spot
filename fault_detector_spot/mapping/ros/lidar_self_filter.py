import math
from typing import Sequence

import numpy as np
import rclpy
import tf2_ros
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray


def rotation_matrix_from_quaternion(rotation) -> np.ndarray:
    norm = math.sqrt(
        rotation.x * rotation.x
        + rotation.y * rotation.y
        + rotation.z * rotation.z
        + rotation.w * rotation.w
    )

    if norm == 0.0:
        raise ValueError("Transform contains an invalid quaternion")

    x = rotation.x / norm
    y = rotation.y / norm
    z = rotation.z / norm
    w = rotation.w / norm

    return np.array(
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ],
        dtype=np.float64,
    )


def validate_box(value: Sequence[float]) -> np.ndarray:
    bounds = np.asarray(value, dtype=np.float64)

    if bounds.shape != (6,):
        raise ValueError("arm_box must contain exactly six values")

    if (
        bounds[0] >= bounds[1]
        or bounds[2] >= bounds[3]
        or bounds[4] >= bounds[5]
    ):
        raise ValueError("arm_box contains invalid minimum and maximum values")

    return bounds


def points_inside_box(points: np.ndarray, bounds: np.ndarray) -> np.ndarray:
    return (
        (points[:, 0] >= bounds[0])
        & (points[:, 0] <= bounds[1])
        & (points[:, 1] >= bounds[2])
        & (points[:, 1] <= bounds[3])
        & (points[:, 2] >= bounds[4])
        & (points[:, 2] <= bounds[5])
    )


class LidarSelfFilter(Node):
    def __init__(self) -> None:
        super().__init__("lidar_self_filter")

        self.declare_parameter("input_topic", "/velodyne/points")
        self.declare_parameter("output_topic", "/velodyne/points_filtered")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter(
            "arm_box",
            [0.25, 0.70, -0.23, 0.23, 0.00, 1.10],
        )
        self.declare_parameter("tf_timeout_sec", 0.25)
        self.declare_parameter("publish_markers", True)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.tf_timeout_sec = float(
            self.get_parameter("tf_timeout_sec").value
        )
        self.last_warning_ns = -2_000_000_000
        self.last_stats_ns = -2_000_000_000

        validate_box(self.get_parameter("arm_box").value)

        cloud_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        marker_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.cloud_publisher = self.create_publisher(
            PointCloud2,
            output_topic,
            cloud_qos,
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            "~/exclusion_boxes",
            marker_qos,
        )
        self.cloud_subscription = self.create_subscription(
            PointCloud2,
            input_topic,
            self.cloud_callback,
            cloud_qos,
        )
        self.marker_timer = self.create_timer(0.2, self.publish_arm_marker)

    def get_arm_box(self) -> np.ndarray:
        return validate_box(self.get_parameter("arm_box").value)

    def warn_throttled(self, message: str) -> None:
        now_ns = self.get_clock().now().nanoseconds

        if now_ns - self.last_warning_ns >= 2_000_000_000:
            self.get_logger().warning(message)
            self.last_warning_ns = now_ns

    def validate_cloud(self, message: PointCloud2) -> None:
        if message.height != 1:
            raise ValueError("Only unorganized point clouds are supported")

        if message.is_bigendian:
            raise ValueError("Big endian point clouds are not supported")

        if message.point_step != 12:
            raise ValueError(
                f"Expected point_step 12, received {message.point_step}"
            )

        if message.row_step != message.width * message.point_step:
            raise ValueError("Point cloud contains unsupported row padding")

        if len(message.data) != message.width * message.point_step:
            raise ValueError("Point cloud data length is inconsistent")

        fields = {field.name: field for field in message.fields}

        for name, offset in {"x": 0, "y": 4, "z": 8}.items():
            field = fields.get(name)

            if field is None:
                raise ValueError(f"Point cloud is missing field {name}")

            if (
                field.offset != offset
                or field.datatype != PointField.FLOAT32
                or field.count != 1
            ):
                raise ValueError(f"Point cloud field {name} is unsupported")

    def cloud_callback(self, message: PointCloud2) -> None:
        try:
            self.validate_cloud(message)
            arm_box = self.get_arm_box()
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                message.header.frame_id,
                Time.from_msg(message.header.stamp),
                timeout=Duration(seconds=self.tf_timeout_sec),
            )
        except Exception as exception:
            self.warn_throttled(str(exception))
            return

        sensor_points = np.frombuffer(
            message.data,
            dtype=np.dtype("<f4"),
        ).reshape((-1, 3))

        rotation = rotation_matrix_from_quaternion(
            transform.transform.rotation
        )
        translation = np.array(
            [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ],
            dtype=np.float64,
        )
        base_points = (
            sensor_points.astype(np.float64, copy=False) @ rotation.T
            + translation
        )
        excluded = points_inside_box(base_points, arm_box)
        filtered_points = np.ascontiguousarray(
            sensor_points[~excluded],
            dtype=np.dtype("<f4"),
        )

        output = PointCloud2()
        output.header = message.header
        output.height = 1
        output.width = filtered_points.shape[0]
        output.fields = list(message.fields)
        output.is_bigendian = False
        output.point_step = 12
        output.row_step = output.width * output.point_step
        output.data = filtered_points.tobytes()
        output.is_dense = message.is_dense
        self.cloud_publisher.publish(output)

        now_ns = self.get_clock().now().nanoseconds

        if now_ns - self.last_stats_ns >= 2_000_000_000:
            self.get_logger().info(
                f"LiDAR filter: input={message.width}, "
                f"removed={int(np.count_nonzero(excluded))}, "
                f"output={output.width}"
            )
            self.last_stats_ns = now_ns

    def publish_arm_marker(self) -> None:
        if not bool(self.get_parameter("publish_markers").value):
            return

        try:
            bounds = self.get_arm_box()
        except ValueError as exception:
            self.warn_throttled(str(exception))
            return

        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "lidar_self_filter"
        marker.id = 1
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.frame_locked = True
        marker.pose.position.x = float((bounds[0] + bounds[1]) / 2.0)
        marker.pose.position.y = float((bounds[2] + bounds[3]) / 2.0)
        marker.pose.position.z = float((bounds[4] + bounds[5]) / 2.0)
        marker.pose.orientation.w = 1.0
        marker.scale.x = float(bounds[1] - bounds[0])
        marker.scale.y = float(bounds[3] - bounds[2])
        marker.scale.z = float(bounds[5] - bounds[4])
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.0
        marker.color.a = 0.25

        markers = MarkerArray()
        markers.markers = [marker]
        self.marker_publisher.publish(markers)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarSelfFilter()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()