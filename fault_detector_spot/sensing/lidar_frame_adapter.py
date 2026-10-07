"""Re-express Spot lidar clouds in their physical sensor frame using TF."""

import math
import time

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import JumpThreshold
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, TransformException, TransformListener
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

from fault_detector_spot.shared.ros.tf_transforms import transform_to_pose_data


def validate_lidar_cloud(cloud, max_points=100000):
    """Accept only the packed XYZ32 layout emitted by the current Spot driver."""
    fields = [(f.name, f.offset, f.datatype, f.count) for f in cloud.fields]
    expected = [(name, index * 4, PointField.FLOAT32, 1)
                for index, name in enumerate("xyz")]
    if (cloud.height != 1 or not 0 < cloud.width <= max_points
            or cloud.is_bigendian or cloud.point_step != 12
            or cloud.row_step != cloud.width * 12
            or len(cloud.data) != cloud.row_step or fields != expected):
        raise ValueError("Expected a bounded, nonempty packed little-endian XYZ32 cloud")
    if not cloud.header.frame_id.strip():
        raise ValueError("Lidar cloud frame must not be empty")


def transform_lidar_cloud(cloud, transform, max_points=100000):
    """Transform coordinates and frame together, preserving acquisition time."""
    validate_lidar_cloud(cloud, max_points)
    if (not transform.header.frame_id.strip()
            or transform.child_frame_id != cloud.header.frame_id):
        raise ValueError("Lidar transform frames do not match the cloud")
    transform_to_pose_data(transform)  # Reject invalid rotations/translations.
    return do_transform_cloud(cloud, transform)


class LidarFrameAdapter:
    """Own sensor conversion only, with no mapping or motion dependencies."""

    def __init__(self, node):
        self.node = node
        descriptor = ParameterDescriptor(read_only=True)
        self.sensor_frame = node.declare_parameter(
            "sensor_frame", "lidar_sensor", descriptor).value
        self.max_cloud_age_sec = node.declare_parameter(
            "max_cloud_age_sec", 0.5, descriptor).value
        self.max_points = node.declare_parameter(
            "max_points", 100000, descriptor).value
        max_rate_hz = node.declare_parameter(
            "max_rate_hz", 5.0, descriptor).value
        if (not self.sensor_frame or self.sensor_frame != self.sensor_frame.strip()
                or self.sensor_frame.startswith("/")):
            raise ValueError("sensor_frame must be a nonempty TF frame without a leading slash")
        if (type(self.max_points) is not int or self.max_points <= 0
                or not math.isfinite(self.max_cloud_age_sec) or self.max_cloud_age_sec <= 0
                or not math.isfinite(max_rate_hz) or max_rate_hz <= 0):
            raise ValueError("Cloud age, point limit and update rate must be positive and finite")
        if node.resolve_topic_name("input") == node.resolve_topic_name("output"):
            raise ValueError("Lidar input and output topics must be different")
        self._period_sec = 1.0 / max_rate_hz
        self._last_attempt = -math.inf
        self._buffer = Buffer()
        self._clock_jump = node.get_clock().create_jump_callback(
            JumpThreshold(
                min_forward=None, min_backward=Duration(nanoseconds=-1), on_clock_change=True,
            ),
            post_callback=lambda _jump: self._buffer.clear(),
        )
        self._listener = TransformListener(self._buffer, node)
        self._publisher = node.create_publisher(
            PointCloud2, "output", QoSProfile(
                depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            ),
        )
        self._subscription = node.create_subscription(
            PointCloud2, "input", self.receive_cloud, QoSProfile(
                depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            ),
        )

    def _check_age(self, cloud):
        stamp = Time.from_msg(cloud.header.stamp)
        now_ns = self.node.get_clock().now().nanoseconds
        age_sec = (now_ns - stamp.nanoseconds) / 1e9
        if (now_ns <= 0 or stamp.nanoseconds <= 0
                or not -0.05 <= age_sec <= self.max_cloud_age_sec):
            raise ValueError("Lidar acquisition time is missing, stale or in the future")
        return stamp

    def receive_cloud(self, cloud):
        now = time.monotonic()
        if now - self._last_attempt < self._period_sec:
            return
        self._last_attempt = now
        try:
            validate_lidar_cloud(cloud, self.max_points)
            stamp = self._check_age(cloud)
            # No waiting, latest-TF fallback, or retained cloud backlog.
            transform = self._buffer.lookup_transform(
                self.sensor_frame, cloud.header.frame_id, stamp, timeout=Duration(),
            )
            output = transform_lidar_cloud(cloud, transform, self.max_points)
            self._check_age(output)
        except (ValueError, TypeError, AssertionError, TransformException) as exception:
            self.node.get_logger().warning(
                f"Skipping lidar cloud: {exception}", throttle_duration_sec=2.0,
            )
            return
        self._publisher.publish(output)

    def destroy(self):
        if self._clock_jump is not None:
            self._clock_jump.unregister()
            self._clock_jump = None
        if self._listener is not None:
            self._listener.unregister()
            self._listener = None


def main(args=None):
    rclpy.init(args=args)
    node = Node("lidar_frame_adapter")
    adapter = None
    try:
        adapter = LidarFrameAdapter(node)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if adapter is not None:
            adapter.destroy()
        node.destroy_node()
        rclpy.try_shutdown()
