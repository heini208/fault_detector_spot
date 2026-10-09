"""Re-express Spot lidar clouds in their physical sensor frame using TF."""

import math
import time

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import JumpThreshold
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

from fault_detector_spot.shared.ros.tf_transforms import transform_to_pose_data


STOP_SERVICE = "/fault_detector/lidar_adapter/stop"


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
        # Live SDK clouds arrive about 0.4 s old, with observed spikes to 0.56 s.
        self.max_cloud_age_sec = node.declare_parameter(
            "max_cloud_age_sec", 0.75, descriptor).value
        self.max_points = node.declare_parameter(
            "max_points", 100000, descriptor).value
        # Leave headroom above the driver's 10 Hz cadence: capping at Nav2's
        # 0.2 s observation deadline drops jittered arrivals and makes scans stale.
        max_rate_hz = node.declare_parameter(
            "max_rate_hz", 20.0, descriptor).value
        collision_max_rate_hz = node.declare_parameter(
            "collision_max_rate_hz", 5.0, descriptor).value
        if (not self.sensor_frame or self.sensor_frame != self.sensor_frame.strip()
                or self.sensor_frame.startswith("/")):
            raise ValueError("sensor_frame must be a nonempty TF frame without a leading slash")
        if (type(self.max_points) is not int or self.max_points <= 0
                or not math.isfinite(self.max_cloud_age_sec) or self.max_cloud_age_sec <= 0
                or not math.isfinite(max_rate_hz) or max_rate_hz <= 0
                or not math.isfinite(collision_max_rate_hz) or collision_max_rate_hz <= 0):
            raise ValueError("Cloud age, point limit and update rate must be positive and finite")
        topics = {node.resolve_topic_name(topic)
                  for topic in ("input", "output", "collision_output")}
        if len(topics) != 3:
            raise ValueError("Lidar input and output topics must all be different")
        self._period_sec = 1.0 / max_rate_hz
        self._last_attempt = -math.inf
        self._collision_period_sec = 1.0 / collision_max_rate_hz
        self._last_collision_publish = -math.inf
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
        self._collision_publisher = node.create_publisher(
            PointCloud2, "collision_output", QoSProfile(
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
        self.stop_requested = False
        self._stop_service = node.create_service(Trigger, STOP_SERVICE, self.request_stop)

    def request_stop(self, _request, response):
        self.stop_requested = True
        response.success = True
        response.message = "Lidar adapter stopping"
        return response

    def _check_age(self, cloud, stage):
        stamp = Time.from_msg(cloud.header.stamp)
        now_ns = self.node.get_clock().now().nanoseconds
        age_sec = (now_ns - stamp.nanoseconds) / 1e9
        if (now_ns <= 0 or stamp.nanoseconds <= 0
                or not -0.05 <= age_sec <= self.max_cloud_age_sec):
            raise ValueError(
                "Lidar acquisition time is missing, stale or in the future "
                f"(stage={stage}, age={age_sec:+.3f} s, "
                f"allowed=-0.050..{self.max_cloud_age_sec:.3f} s, "
                f"stamp_ns={stamp.nanoseconds}, now_ns={now_ns})"
            )
        return stamp

    def receive_cloud(self, cloud):
        now = time.monotonic()
        if now - self._last_attempt < self._period_sec:
            return
        self._last_attempt = now
        try:
            validate_lidar_cloud(cloud, self.max_points)
            stamp = self._check_age(cloud, "input")
            # No waiting, latest-TF fallback, or retained cloud backlog.
            transform = self._buffer.lookup_transform(
                self.sensor_frame, cloud.header.frame_id, stamp, timeout=Duration(),
            )
            output = transform_lidar_cloud(cloud, transform, self.max_points)
            self._check_age(output, "after transform")
        except (ValueError, TypeError, AssertionError, TransformException) as exception:
            self.node.get_logger().warning(
                f"Skipping lidar cloud: {exception}", throttle_duration_sec=2.0,
            )
            return
        self._publisher.publish(output)
        # MoveIt's occupancy updater keeps its existing workload budget without
        # throttling navigation. Reuse this scan; never retain it for later.
        now = time.monotonic()
        if now - self._last_collision_publish >= self._collision_period_sec:
            self._collision_publisher.publish(output)
            self._last_collision_publish = now

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
    executor = SingleThreadedExecutor()
    adapter = None
    try:
        adapter = LidarFrameAdapter(node)
        executor.add_node(node)
        while rclpy.ok() and not adapter.stop_requested:
            # The service handler sends its response before spin_once returns.
            executor.spin_once(timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        if adapter is not None:
            adapter.destroy()
        node.destroy_node()
        rclpy.try_shutdown()
