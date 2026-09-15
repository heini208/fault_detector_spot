"""Live hand-depth surface sensing used by arm execution."""

from collections import deque
from copy import deepcopy
from threading import RLock
import math
import time

from fault_detector_msgs.msg import SensorAttachmentState
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
import tf2_ros

from fault_detector_spot.inspection.geometry.surface_normal import (
    SurfaceNormalEstimate,
    estimate_surface_normal,
)
from fault_detector_spot.inspection.geometry.depth_point_cloud import (
    create_organized_depth_point_cloud,
)
from fault_detector_spot.inspection.model.models import ImagePoint, PoseData
from fault_detector_spot.inspection.model.sensor_models import (
    BARE_HAND_MOTION_ID,
    sensor_probe_frame,
)
from fault_detector_spot.inspection.sensing.live_surface_distance import (
    measure_probe_surface_distance,
)
from fault_detector_spot.inspection.setup.reference_view_depth_projection import (
    project_reference_pixel,
)
from fault_detector_spot.shared.ros.qos_profiles import APPLICATION_STATE_QOS
from fault_detector_spot.shared.ros.tf_transforms import lookup_pose_data
from fault_detector_spot.shared.runtime_source import RuntimeSource


HAND_DEPTH_HISTORY_MAX_SAMPLES = 32
MAX_HAND_DEPTH_AGE_SEC = 0.5
SURFACE_ORIENTATION_WINDOW_RADIUS_PX = 16
MINIMUM_SURFACE_ORIENTATION_CAMERA_DISTANCE_M = 0.290
SENSOR_ATTACHMENT_TOPIC = "fault_detector/application/sensor_attachment_state"


class ProbeSurfaceSource(RuntimeSource):
    """Cache hand depth and active attachment for surface measurement."""

    def __init__(self, node):
        if node is None:
            raise RuntimeError("ProbeSurfaceSource requires a ROS node")
        self.node = node
        self._lock = RLock()
        self._hand_depth_history = deque(
            maxlen=HAND_DEPTH_HISTORY_MAX_SAMPLES
        )
        self._hand_depth_camera_info = None
        self._attachment_state = None
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(
            self._tf_buffer,
            node,
        )
        self._hand_depth_subscription = node.create_subscription(
            Image,
            "/depth_registered/hand/image",
            self._receive_hand_depth,
            qos_profile_sensor_data,
        )
        self._hand_depth_camera_info_subscription = node.create_subscription(
            CameraInfo,
            "/depth_registered/hand/camera_info",
            self._receive_hand_depth_camera_info,
            qos_profile_sensor_data,
        )
        self._attachment_subscription = node.create_subscription(
            SensorAttachmentState,
            SENSOR_ATTACHMENT_TOPIC,
            self._receive_attachment_state,
            APPLICATION_STATE_QOS,
        )

    def active_attachment(self):
        """Return the effective motion sensor and attachment revision."""
        with self._lock:
            state = deepcopy(self._attachment_state)
        if state is None:
            raise ValueError("Sensor attachment state is unavailable")
        if state.status in {
            SensorAttachmentState.STATUS_CONFIRMATION_PENDING,
            SensorAttachmentState.STATUS_NO_SENSOR,
        }:
            raise ValueError("Sensor attachment confirmation is pending")
        if state.status != SensorAttachmentState.STATUS_ACTIVE:
            raise ValueError("Sensor attachment state is invalid")
        sensor_id = state.active_sensor_id.strip() or BARE_HAND_MOTION_ID
        return sensor_id, int(state.attachment_revision)

    def latest_hand_depth(
        self,
        maximum_age_sec: float = MAX_HAND_DEPTH_AGE_SEC,
    ):
        """Return the newest fresh registered hand depth and camera info."""
        maximum_age_sec = float(maximum_age_sec)
        if not math.isfinite(maximum_age_sec) or maximum_age_sec <= 0.0:
            raise ValueError("Maximum hand-depth age must be positive")

        with self._lock:
            camera_info = deepcopy(self._hand_depth_camera_info)
            history = tuple(self._hand_depth_history)
        if camera_info is None:
            raise ValueError(
                "No registered hand-depth camera info is available"
            )
        if not history:
            raise ValueError(
                "No registered hand-depth image is available"
            )

        receipt_time, depth_image = history[-1]
        age = time.monotonic() - receipt_time
        if age < -1e-9 or age > maximum_age_sec:
            raise ValueError(
                "Registered hand-depth image is stale: "
                f"age={age:.3f}s, max_age={maximum_age_sec:.3f}s"
            )
        return deepcopy(depth_image), camera_info

    def surface_normal(
        self,
        maximum_age_sec: float = MAX_HAND_DEPTH_AGE_SEC,
        window_radius_px: int = SURFACE_ORIENTATION_WINDOW_RADIUS_PX,
    ) -> SurfaceNormalEstimate:
        """Return a live local surface-normal estimate at image center."""
        if (
            isinstance(window_radius_px, bool)
            or not isinstance(window_radius_px, int)
            or window_radius_px <= 0
        ):
            raise ValueError("Surface orientation window radius must be positive")

        depth_image, camera_info = self.latest_hand_depth(maximum_age_sec)
        point_cloud = create_organized_depth_point_cloud(
            depth_image, camera_info,
        )
        center = ImagePoint(
            u=int(depth_image.width) // 2,
            v=int(depth_image.height) // 2,
        )
        try:
            projected = project_reference_pixel(
                center,
                depth_image,
                camera_info,
                search_radius_px=window_radius_px,
                point_cloud=point_cloud,
                rgb_size=(
                    int(depth_image.width),
                    int(depth_image.height),
                ),
            )
        except ValueError as exception:
            if "No valid depth within" not in str(exception):
                raise
            raise ValueError(
                "No valid registered hand depth was found in the center "
                f"{window_radius_px} px window. The surface may be too "
                "close or too far from the gripper depth camera."
            ) from exception

        camera_distance_m = float(projected.depth_m)
        if camera_distance_m < MINIMUM_SURFACE_ORIENTATION_CAMERA_DISTANCE_M:
            raise ValueError(
                "Surface is too close for reliable orientation: "
                f"camera distance {camera_distance_m:.3f} m, minimum "
                f"{MINIMUM_SURFACE_ORIENTATION_CAMERA_DISTANCE_M:.3f} m"
            )

        try:
            return estimate_surface_normal(
                projected,
                depth_image,
                camera_info,
                neighborhood_radius_px=window_radius_px,
                maximum_neighborhood_radius_px=window_radius_px,
                point_cloud=point_cloud,
            )
        except ValueError as exception:
            raise ValueError(
                "Unable to fit a reliable front surface plane: "
                f"{exception}"
            ) from exception

    def surface_distance_samples(
        self,
        sensor_id: str,
        receipt_not_before: float = 0.0,
        maximum_age_sec: float = MAX_HAND_DEPTH_AGE_SEC,
        minimum_samples: int = 1,
    ):
        """Return fresh probe-axis surface-distance samples."""
        sensor_id = str(sensor_id).strip()
        if not sensor_id:
            raise ValueError("Sensor ID must not be empty")
        if not math.isfinite(float(receipt_not_before)):
            raise ValueError("Surface sample receipt threshold must be finite")
        if not math.isfinite(float(maximum_age_sec)) or maximum_age_sec <= 0.0:
            raise ValueError("Maximum hand-depth age must be positive")
        if (
            isinstance(minimum_samples, bool)
            or not isinstance(minimum_samples, int)
            or minimum_samples < 1
        ):
            raise ValueError("Minimum surface sample count must be positive")

        with self._lock:
            camera_info = deepcopy(self._hand_depth_camera_info)
            history = tuple(self._hand_depth_history)
        if camera_info is None:
            raise ValueError(
                "No registered hand-depth camera info is available"
            )

        samples = []
        errors = []
        now = time.monotonic()
        post_start_count = 0
        eligible_count = 0
        for receipt_time, depth_image in history:
            if receipt_time + 1e-9 < receipt_not_before:
                continue
            post_start_count += 1
            age = now - receipt_time
            if age < -1e-9 or age > maximum_age_sec:
                continue
            eligible_count += 1
            try:
                stamp = Time.from_msg(depth_image.header.stamp)
                if stamp.nanoseconds <= 0:
                    raise ValueError("Registered hand-depth timestamp is empty")
                depth_frame = (
                    depth_image.header.frame_id.strip()
                    or camera_info.header.frame_id.strip()
                )
                if not depth_frame:
                    raise ValueError("Registered hand-depth frame is empty")
                probe_to_camera = self._lookup_pose(
                    sensor_probe_frame(sensor_id),
                    depth_frame,
                    lookup_time=stamp,
                )
                samples.append(
                    measure_probe_surface_distance(
                        depth_image,
                        camera_info,
                        probe_to_camera,
                    )
                )
            except Exception as exception:
                errors.append(str(exception))

        if len(samples) < minimum_samples:
            if errors:
                detail = (
                    f"{eligible_count} eligible frame(s), "
                    f"{len(errors)} rejected; last error: {errors[-1]}"
                )
            else:
                newest_age = now - history[-1][0] if history else None
                newest_detail = (
                    f"{newest_age:.3f}s"
                    if newest_age is not None
                    else "none"
                )
                detail = (
                    "no eligible depth frames "
                    f"(buffered={len(history)}, "
                    f"post_start={post_start_count}, "
                    f"within_age={eligible_count}, "
                    f"newest_age={newest_detail}, "
                    f"max_age={maximum_age_sec:.3f}s)"
                )
            raise ValueError(
                "Need at least "
                f"{minimum_samples} fresh registered hand-depth sample"
                f"{'s' if minimum_samples != 1 else ''}: {detail}"
            )
        return tuple(samples)

    def _lookup_pose(
        self,
        target_frame: str,
        source_frame: str,
        lookup_time=None,
    ) -> PoseData:
        return lookup_pose_data(
            self._tf_buffer,
            target_frame,
            source_frame,
            lookup_time=lookup_time,
        )

    def _receive_hand_depth(self, message: Image) -> None:
        with self._lock:
            self._hand_depth_history.append(
                (time.monotonic(), deepcopy(message))
            )

    def _receive_hand_depth_camera_info(self, message: CameraInfo) -> None:
        with self._lock:
            self._hand_depth_camera_info = deepcopy(message)

    def _receive_attachment_state(self, message: SensorAttachmentState) -> None:
        with self._lock:
            self._attachment_state = deepcopy(message)

    def destroy(self) -> None:
        subscriptions = (
            "_hand_depth_subscription",
            "_hand_depth_camera_info_subscription",
            "_attachment_subscription",
        )
        for attribute in subscriptions:
            subscription = getattr(self, attribute, None)
            if subscription is not None:
                self.node.destroy_subscription(subscription)
                setattr(self, attribute, None)
        if self._tf_listener is not None:
            self._tf_listener.unregister()
            self._tf_listener = None
        with self._lock:
            self._hand_depth_history.clear()
            self._hand_depth_camera_info = None
            self._attachment_state = None


__all__ = [
    "MINIMUM_SURFACE_ORIENTATION_CAMERA_DISTANCE_M",
    "ProbeSurfaceSource",
    "SURFACE_ORIENTATION_WINDOW_RADIUS_PX",
]
