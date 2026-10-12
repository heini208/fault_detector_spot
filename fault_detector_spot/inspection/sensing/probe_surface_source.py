"""Live hand-depth surface sensing used by arm execution."""

from collections import deque
from copy import deepcopy
from threading import RLock
import math
import time

from fault_detector_msgs.msg import SensorAttachmentState
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image

from fault_detector_spot.inspection.geometry.surface_normal import (
    SurfaceNormalEstimate,
)
from fault_detector_spot.inspection.sensing.live_surface_plane import (
    MINIMUM_SURFACE_ORIENTATION_CAMERA_DISTANCE_M,
    SURFACE_ORIENTATION_MAXIMUM_FIT_RADIUS_PX,
    SURFACE_ORIENTATION_MINIMUM_SAMPLE_COUNT,
    SURFACE_ORIENTATION_WINDOW_RADIUS_PX,
    estimate_live_surface_normal,
)
from fault_detector_spot.shared.geometry.models import (
    PoseData,
)
from fault_detector_spot.inspection.model.sensor_models import (
    BARE_HAND_MOTION_ID,
    sensor_probe_frame,
)
from fault_detector_spot.inspection.sensing.live_surface_distance import (
    measure_probe_surface_distance,
)
from fault_detector_spot.shared.ros.qos_profiles import APPLICATION_STATE_QOS
from fault_detector_spot.shared.ros.tf_transforms import lookup_pose_data
from fault_detector_spot.shared.runtime_source import RuntimeSource


HAND_DEPTH_HISTORY_MAX_SAMPLES = 32
MAX_HAND_DEPTH_AGE_SEC = 0.5
SENSOR_ATTACHMENT_TOPIC = "fault_detector/application/sensor_attachment_state"


class ProbeSurfaceSource(RuntimeSource):
    """Cache hand depth and active attachment for surface measurement."""

    def __init__(self, node, tf_buffer):
        if node is None:
            raise RuntimeError("ProbeSurfaceSource requires a ROS node")
        self.node = node
        self._lock = RLock()
        self._callback_group = MutuallyExclusiveCallbackGroup()
        self._hand_depth_history = deque(
            maxlen=HAND_DEPTH_HISTORY_MAX_SAMPLES
        )
        self._surface_distance_cache = {}
        self._hand_depth_camera_info = None
        self._attachment_state = None
        self._tf_buffer = tf_buffer
        self._hand_depth_subscription = node.create_subscription(
            Image,
            "/depth_registered/hand/image",
            self._receive_hand_depth,
            qos_profile_sensor_data,
            callback_group=self._callback_group,
        )
        self._hand_depth_camera_info_subscription = node.create_subscription(
            CameraInfo,
            "/depth_registered/hand/camera_info",
            self._receive_hand_depth_camera_info,
            qos_profile_sensor_data,
            callback_group=self._callback_group,
        )
        self._attachment_subscription = node.create_subscription(
            SensorAttachmentState,
            SENSOR_ATTACHMENT_TOPIC,
            self._receive_attachment_state,
            APPLICATION_STATE_QOS,
            callback_group=self._callback_group,
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
        receipt_not_before: float = 0.0,
    ):
        """Return the newest fresh registered hand depth and camera info."""
        maximum_age_sec = float(maximum_age_sec)
        receipt_not_before = float(receipt_not_before)
        if not math.isfinite(maximum_age_sec) or maximum_age_sec <= 0.0:
            raise ValueError("Maximum hand-depth age must be positive")
        if not math.isfinite(receipt_not_before):
            raise ValueError("Hand-depth receipt threshold must be finite")

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

        eligible = [
            item
            for item in history
            if item[0] + 1e-9 >= receipt_not_before
        ]
        if not eligible:
            raise ValueError(
                "No registered hand-depth image has been received after "
                f"the required start time {receipt_not_before:.6f}"
            )

        receipt_time, depth_image = eligible[-1]
        age = time.monotonic() - receipt_time
        if age < -1e-9 or age > maximum_age_sec:
            publishers = self.node.count_publishers("/depth_registered/hand/image")
            raise ValueError(
                "Registered hand-depth image is stale: "
                f"age={age:.3f}s, max_age={maximum_age_sec:.3f}s; "
                f"discovered publishers={publishers}, buffered frames={len(history)}; "
                "no depth callback has refreshed the cache within the age limit"
            )
        return deepcopy(depth_image), camera_info

    def surface_normal(
        self,
        maximum_age_sec: float = MAX_HAND_DEPTH_AGE_SEC,
        window_radius_px: int = SURFACE_ORIENTATION_WINDOW_RADIUS_PX,
        receipt_not_before: float = 0.0,
    ) -> SurfaceNormalEstimate:
        """Return a live local surface normal from the lower-middle image patch."""
        if float(receipt_not_before) > 0.0:
            depth_image, camera_info = self.latest_hand_depth(
                maximum_age_sec,
                receipt_not_before=receipt_not_before,
            )
        else:
            depth_image, camera_info = self.latest_hand_depth(
                maximum_age_sec,
            )
        return estimate_live_surface_normal(
            depth_image,
            camera_info,
            window_radius_px=window_radius_px,
        )

    def surface_distance_samples(
        self,
        sensor_id: str,
        receipt_not_before: float = 0.0,
        maximum_age_sec: float = MAX_HAND_DEPTH_AGE_SEC,
        minimum_samples: int = 1,
        maximum_latest_age_sec: float | None = None,
    ):
        """Return fresh probe-axis surface-distance samples."""
        sensor_id = str(sensor_id).strip()
        if not sensor_id:
            raise ValueError("Sensor ID must not be empty")
        if not math.isfinite(float(receipt_not_before)):
            raise ValueError("Surface sample receipt threshold must be finite")
        if not math.isfinite(float(maximum_age_sec)) or maximum_age_sec <= 0.0:
            raise ValueError("Maximum hand-depth age must be positive")
        if maximum_latest_age_sec is not None and (
            not math.isfinite(float(maximum_latest_age_sec))
            or maximum_latest_age_sec <= 0.0
        ):
            raise ValueError("Maximum latest hand-depth age must be positive")
        if (
            isinstance(minimum_samples, bool)
            or not isinstance(minimum_samples, int)
            or minimum_samples < 1
        ):
            raise ValueError("Minimum surface sample count must be positive")

        with self._lock:
            camera_info = deepcopy(self._hand_depth_camera_info)
            history = tuple(self._hand_depth_history)
            attachment_state = deepcopy(self._attachment_state)
        if camera_info is None:
            raise ValueError(
                "No registered hand-depth camera info is available"
            )

        samples = []
        valid_receipts = []
        errors = []
        now = time.monotonic()
        context = (
            sensor_id,
            None if attachment_state is None else (
                attachment_state.status,
                attachment_state.active_sensor_id,
                attachment_state.attachment_revision,
            ),
            camera_info.header.frame_id,
            camera_info.width,
            camera_info.height,
            tuple(camera_info.k),
            tuple(camera_info.p),
        )
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
                # Check timestamped TF on every poll; only reuse the expensive
                # depth fit when its image, calibration, attachment and pose
                # are unchanged. Failed fits/TF lookups must remain retryable.
                cache_key = (context, probe_to_camera)
                with self._lock:
                    cached = self._surface_distance_cache.get(id(depth_image))
                if cached is not None and cached[0] == cache_key:
                    sample = deepcopy(cached[1])
                else:
                    sample = measure_probe_surface_distance(
                        depth_image,
                        camera_info,
                        probe_to_camera,
                    )
                    with self._lock:
                        if any(
                            image is depth_image
                            for _, image in self._hand_depth_history
                        ):
                            self._surface_distance_cache[id(depth_image)] = (
                                deepcopy(cache_key), deepcopy(sample),
                            )
                samples.append(sample)
                valid_receipts.append(receipt_time)
            except Exception as exception:
                errors.append(str(exception))

        if maximum_latest_age_sec is not None and valid_receipts:
            # Plane fitting itself takes time. Freshness must be checked when
            # returning a measurement, not only before processing the batch.
            latest_age = time.monotonic() - max(valid_receipts)
            if latest_age < -1e-9 or latest_age > maximum_latest_age_sec:
                raise ValueError(
                    "Latest valid registered hand-depth sample is stale: "
                    f"age={latest_age:.3f}s, "
                    f"max_age={maximum_latest_age_sec:.3f}s"
                )

        if len(samples) < minimum_samples:
            newest_age = now - history[-1][0] if history else None
            newest_detail = (
                f"{newest_age:.3f}s"
                if newest_age is not None
                else "none"
            )
            detail = (
                f"buffered={len(history)}, "
                f"post_start={post_start_count}, "
                f"within_age={eligible_count}, "
                f"valid={len(samples)}, "
                f"rejected={len(errors)}, "
                f"newest_age={newest_detail}, "
                f"max_age={maximum_age_sec:.3f}s"
            )
            if errors:
                detail += f"; last rejection: {errors[-1]}"
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
            buffered_images = {
                id(image) for _, image in self._hand_depth_history
            }
            self._surface_distance_cache = {
                key: cached
                for key, cached in self._surface_distance_cache.items()
                if key in buffered_images
            }

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
        with self._lock:
            self._hand_depth_history.clear()
            self._surface_distance_cache.clear()
            self._hand_depth_camera_info = None
            self._attachment_state = None


__all__ = [
    "MINIMUM_SURFACE_ORIENTATION_CAMERA_DISTANCE_M",
    "ProbeSurfaceSource",
    "SURFACE_ORIENTATION_MAXIMUM_FIT_RADIUS_PX",
    "SURFACE_ORIENTATION_MINIMUM_SAMPLE_COUNT",
    "SURFACE_ORIENTATION_WINDOW_RADIUS_PX",
]
