"""Provide authoritative runtime state for probe setup movement."""

from collections import deque
from copy import deepcopy
import math
from threading import RLock
import time

from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME
from fault_detector_msgs.msg import TagElementArray
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import CameraInfo, Image
import tf2_ros

from fault_detector_spot.inspection.model.models import (
    ImagePoint,
    PoseData,
    QuaternionData,
    Vector3Data,
)
from fault_detector_spot.inspection.model.sensor_models import (
    sensor_probe_frame,
)
from fault_detector_spot.inspection.sensing.end_effector_force import (
    EndEffectorForceSample,
)
from fault_detector_spot.inspection.setup.reference_probe_setup import (
    compose_poses,
    relative_pose,
)
from fault_detector_spot.inspection.setup.reference_view_depth_projection import (
    project_reference_pixel,
)
from fault_detector_spot.inspection.setup.stable_tag_pose import (
    TagPoseSample,
    stabilize_tag_pose,
)
from fault_detector_spot.shared.geometry.transforms import pose_to_pose_data


BASE_TAG_MAXIMUM_AGE_SEC = 1.5
BASE_TAG_STABILIZATION_HISTORY_SEC = 4.0
BASE_TAG_HISTORY_MAX_SAMPLES = 64
BASE_TAG_MINIMUM_SPAN_SEC = 0.10
HAND_DEPTH_HISTORY_MAX_SAMPLES = 32
MAX_HAND_DEPTH_AGE_SEC = 0.5
HAND_DEPTH_SEARCH_RADIUS_PX = 16
MINIMUM_HAND_CAMERA_SURFACE_CLEARANCE_M = 0.290
END_EFFECTOR_FORCE_TOPIC = "/status/end_effector_force"
END_EFFECTOR_FORCE_HISTORY_MAX_SAMPLES = 128
MAX_END_EFFECTOR_FORCE_AGE_SEC = 0.25


class ProbeSetupMotionStateSource:
    """Resolve stable tag and probe poses outside the remote UI."""

    def __init__(self, node):
        self.node = node
        self._lock = RLock()
        self._base_tag_histories = {}
        self._hand_depth_history = deque(
            maxlen=HAND_DEPTH_HISTORY_MAX_SAMPLES
        )
        self._hand_depth_camera_info = None
        self._end_effector_force_history = deque(
            maxlen=END_EFFECTOR_FORCE_HISTORY_MAX_SAMPLES
        )
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(
            self._tf_buffer,
            node,
        )
        self._base_tag_subscription = node.create_subscription(
            TagElementArray,
            "fault_detector/state/base_tags",
            self._receive_base_tags,
            10,
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
        self._end_effector_force_subscription = node.create_subscription(
            Vector3Stamped,
            END_EFFECTOR_FORCE_TOPIC,
            self._receive_end_effector_force,
            qos_profile_sensor_data,
        )

    def reference_tag(self, tag_id: int):
        """Return one stabilized authoritative base-camera observation."""
        if isinstance(tag_id, bool) or not isinstance(tag_id, int):
            raise TypeError("Reference tag ID must be an integer")
        with self._lock:
            history = tuple(self._base_tag_histories.get(tag_id, ()))
        samples = []
        messages_by_stamp = {}
        for _receipt_time, stamp_key, tag in history:
            stamp_seconds = float(stamp_key[0]) + float(stamp_key[1]) * 1e-9
            samples.append(
                TagPoseSample(
                    stamp_seconds=stamp_seconds,
                    frame_id=tag.pose.header.frame_id.strip(),
                    pose=pose_to_pose_data(tag.pose.pose),
                )
            )
            messages_by_stamp[stamp_seconds] = tag
        stable = stabilize_tag_pose(
            samples,
            now_seconds=self.node.get_clock().now().nanoseconds * 1e-9,
            maximum_age_sec=BASE_TAG_MAXIMUM_AGE_SEC,
            stabilization_window_sec=BASE_TAG_STABILIZATION_HISTORY_SEC,
            minimum_samples=3,
            minimum_span_sec=BASE_TAG_MINIMUM_SPAN_SEC,
        )
        tag = deepcopy(messages_by_stamp[stable.newest_stamp_seconds])
        tag.pose.header.frame_id = stable.frame_id
        tag.pose.pose.position.x = stable.pose.position.x
        tag.pose.pose.position.y = stable.pose.position.y
        tag.pose.pose.position.z = stable.pose.position.z
        tag.pose.pose.orientation.x = stable.pose.orientation.x
        tag.pose.pose.orientation.y = stable.pose.orientation.y
        tag.pose.pose.orientation.z = stable.pose.orientation.z
        tag.pose.pose.orientation.w = stable.pose.orientation.w
        return tag

    def reference_tag_history(
        self,
        tag_id: int,
        receipt_not_before: float,
        receipt_not_after: float,
    ):
        """Return raw tag observations received inside one capture window."""
        if isinstance(tag_id, bool) or not isinstance(tag_id, int):
            raise TypeError("Reference tag ID must be an integer")
        receipt_not_before = float(receipt_not_before)
        receipt_not_after = float(receipt_not_after)
        if (
            not math.isfinite(receipt_not_before)
            or not math.isfinite(receipt_not_after)
        ):
            raise ValueError(
                "Reference tag receipt bounds must be finite"
            )
        if receipt_not_after < receipt_not_before:
            raise ValueError(
                "Reference tag receipt end must not precede its start"
            )
        with self._lock:
            history = tuple(self._base_tag_histories.get(tag_id, ()))
        return tuple(
            deepcopy(tag)
            for receipt_time, _stamp_key, tag in history
            if (
                receipt_time + 1e-9 >= receipt_not_before
                and receipt_time <= receipt_not_after + 1e-9
            )
        )

    def gravity_aligned_object_pose(self, reference_tag_id: int) -> PoseData:
        """Return the tag-defined object pose in Spot's gravity frame."""
        tag = self.reference_tag(reference_tag_id)
        source_frame = tag.pose.header.frame_id.strip()
        source_to_object = pose_to_pose_data(tag.pose.pose)
        if source_frame == GRAV_ALIGNED_BODY_FRAME_NAME:
            return source_to_object
        gravity_to_source = self._lookup_pose(
            GRAV_ALIGNED_BODY_FRAME_NAME,
            source_frame,
        )
        return compose_poses(gravity_to_source, source_to_object)

    def current_probe_pose_object(
        self,
        reference_tag_id: int,
        sensor_id: str,
    ) -> PoseData:
        """Return the live probe pose expressed in the object tag frame."""
        tag = self.reference_tag(reference_tag_id)
        body_frame = tag.pose.header.frame_id.strip()
        body_to_probe = self._lookup_pose(
            body_frame,
            sensor_probe_frame(sensor_id),
        )
        body_to_object = pose_to_pose_data(tag.pose.pose)
        return relative_pose(body_to_object, body_to_probe)

    def minimum_aligned_probe_distance_m(
        self,
        sensor_id: str,
        minimum_camera_clearance_m: float = MINIMUM_HAND_CAMERA_SURFACE_CLEARANCE_M,
    ) -> float:
        """Return the minimum tip distance that preserves hand-depth range."""
        if not isinstance(sensor_id, str) or not sensor_id.strip():
            raise ValueError("Sensor ID must not be empty")
        if (
            not math.isfinite(float(minimum_camera_clearance_m))
            or minimum_camera_clearance_m <= 0.0
        ):
            raise ValueError("Minimum camera clearance must be positive")
        with self._lock:
            camera_info = deepcopy(self._hand_depth_camera_info)
        if camera_info is None:
            raise ValueError(
                "No registered hand-depth camera info is available"
            )
        camera_frame = camera_info.header.frame_id.strip()
        if not camera_frame:
            raise ValueError("Registered hand-depth frame is empty")
        probe_to_camera = self._lookup_pose(
            sensor_probe_frame(sensor_id.strip()),
            camera_frame,
        )
        required_probe_distance_m = (
            float(minimum_camera_clearance_m)
            + float(probe_to_camera.position.x)
        )
        return max(0.0, required_probe_distance_m)

    def validate_aligned_probe_distance(
        self,
        sensor_id: str,
        aligned_probe_distance_m: float,
        minimum_camera_clearance_m: float = MINIMUM_HAND_CAMERA_SURFACE_CLEARANCE_M,
    ) -> float:
        """Validate a user tip distance against the hand-camera near field."""
        if (
            not math.isfinite(float(aligned_probe_distance_m))
            or aligned_probe_distance_m <= 0.0
        ):
            raise ValueError(
                "Aligned pre-approach probe distance must be positive"
            )
        minimum_probe_distance_m = self.minimum_aligned_probe_distance_m(
            sensor_id,
            minimum_camera_clearance_m,
        )
        if aligned_probe_distance_m + 1e-9 < minimum_probe_distance_m:
            raise ValueError(
                "Aligned pre-approach is too close for registered hand depth: "
                f"this sensor requires at least "
                f"{minimum_probe_distance_m:.3f} m probe-to-surface distance "
                f"to preserve {minimum_camera_clearance_m:.3f} m "
                "camera-to-surface clearance"
            )
        return minimum_probe_distance_m

    def current_hand_camera_surface_clearance_m(self) -> float:
        """Return current center-ray camera-to-surface depth clearance."""
        with self._lock:
            camera_info = deepcopy(self._hand_depth_camera_info)
            history = tuple(self._hand_depth_history)
        if camera_info is None:
            raise ValueError(
                "No registered hand-depth camera info is available"
            )
        fresh_history = self._recent_hand_depth_samples(
            history,
            MAX_HAND_DEPTH_AGE_SEC,
        )
        if not fresh_history:
            raise ValueError(
                "No fresh registered hand-depth image is available"
            )
        depth_image = deepcopy(fresh_history[-1][1])
        center = ImagePoint(
            u=int(depth_image.width) // 2,
            v=int(depth_image.height) // 2,
        )
        projected = project_reference_pixel(
            center,
            depth_image,
            camera_info,
            search_radius_px=HAND_DEPTH_SEARCH_RADIUS_PX,
            rgb_size=(int(depth_image.width), int(depth_image.height)),
        )
        return float(projected.depth_m)

    def require_hand_camera_clearance(
        self,
        minimum_camera_clearance_m: float = MINIMUM_HAND_CAMERA_SURFACE_CLEARANCE_M,
    ) -> float:
        """Require the reached aligned pose to remain in usable ToF range."""
        try:
            clearance_m = self.current_hand_camera_surface_clearance_m()
        except ValueError as exception:
            raise ValueError(
                "Unable to verify hand-camera surface clearance from depth. "
                "If the registered depth is invalid because the hand is too "
                "close to the surface, increase the aligned pre-approach "
                "distance and try again. "
                f"Detail: {exception}"
            ) from exception
        if clearance_m + 1e-9 < minimum_camera_clearance_m:
            raise ValueError(
                "Reached aligned pre-approach is inside the hand ToF near "
                f"field: measured camera clearance {clearance_m:.3f} m, "
                f"required at least {minimum_camera_clearance_m:.3f} m"
            )
        return clearance_m

    def end_effector_force_samples(
        self,
        receipt_not_before: float = 0.0,
        maximum_age_sec: float = MAX_END_EFFECTOR_FORCE_AGE_SEC,
    ):
        """Return fresh hand-frame end-effector force samples."""
        if not math.isfinite(float(receipt_not_before)):
            raise ValueError(
                "End-effector force receipt threshold must be finite"
            )
        if (
            not math.isfinite(float(maximum_age_sec))
            or maximum_age_sec <= 0.0
        ):
            raise ValueError(
                "Maximum end-effector force age must be positive"
            )
        with self._lock:
            history = tuple(self._end_effector_force_history)
        now_receipt_time = time.monotonic()
        samples = []
        for receipt_time, message in history:
            if receipt_time + 1e-9 < receipt_not_before:
                continue
            age_seconds = now_receipt_time - receipt_time
            if age_seconds < -1e-9 or age_seconds > maximum_age_sec:
                continue
            stamp = message.header.stamp
            stamp_seconds = float(stamp.sec) + float(stamp.nanosec) * 1e-9
            if stamp_seconds <= 0.0:
                continue
            frame_id = message.header.frame_id.strip()
            if not frame_id:
                continue
            force = Vector3Data(
                x=float(message.vector.x),
                y=float(message.vector.y),
                z=float(message.vector.z),
            )
            force.validate()
            sample = EndEffectorForceSample(
                force_hand=force,
                stamp_seconds=stamp_seconds,
                receipt_time=receipt_time,
                frame_id=frame_id,
            )
            sample.validate()
            samples.append(sample)
        if not samples:
            raise ValueError(
                "No fresh end-effector force samples are available"
            )
        return tuple(samples)

    def latest_end_effector_force(
        self,
        maximum_age_sec: float = MAX_END_EFFECTOR_FORCE_AGE_SEC,
    ) -> EndEffectorForceSample:
        """Return the newest fresh end-effector force sample."""
        return self.end_effector_force_samples(
            maximum_age_sec=maximum_age_sec
        )[-1]

    @staticmethod
    def _recent_hand_depth_samples(
        history,
        maximum_age_sec: float,
        receipt_not_before: float = 0.0,
    ):
        now_receipt_time = time.monotonic()
        recent = []
        for receipt_time, image in history:
            if receipt_time + 1e-9 < receipt_not_before:
                continue
            age_seconds = now_receipt_time - receipt_time
            if age_seconds < -1e-9 or age_seconds > maximum_age_sec:
                continue
            recent.append((receipt_time, image))
        return tuple(recent)

    def _lookup_pose(
        self,
        target_frame: str,
        source_frame: str,
        lookup_time=None,
    ) -> PoseData:
        transform = self._tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            lookup_time if lookup_time is not None else Time(),
            timeout=Duration(seconds=0.5),
        )
        value = transform.transform
        pose = PoseData(
            position=Vector3Data(
                x=value.translation.x,
                y=value.translation.y,
                z=value.translation.z,
            ),
            orientation=QuaternionData(
                x=value.rotation.x,
                y=value.rotation.y,
                z=value.rotation.z,
                w=value.rotation.w,
            ),
        )
        pose.validate()
        return pose

    def _receive_base_tags(self, message: TagElementArray) -> None:
        receipt_time = time.monotonic()
        with self._lock:
            for tag in message.elements:
                stamp = tag.pose.header.stamp
                stamp_key = (int(stamp.sec), int(stamp.nanosec))
                history = self._base_tag_histories.setdefault(
                    int(tag.id),
                    deque(maxlen=BASE_TAG_HISTORY_MAX_SAMPLES),
                )
                if history and history[-1][1] == stamp_key:
                    continue
                history.append(
                    (receipt_time, stamp_key, deepcopy(tag))
                )

    def _receive_hand_depth(self, message: Image) -> None:
        with self._lock:
            self._hand_depth_history.append(
                (time.monotonic(), deepcopy(message))
            )

    def _receive_hand_depth_camera_info(self, message: CameraInfo) -> None:
        with self._lock:
            self._hand_depth_camera_info = deepcopy(message)

    def _receive_end_effector_force(self, message: Vector3Stamped) -> None:
        with self._lock:
            self._end_effector_force_history.append(
                (time.monotonic(), deepcopy(message))
            )

    def close(self) -> None:
        """Destroy ROS resources owned by this state source."""
        self.node.destroy_subscription(self._base_tag_subscription)
        self.node.destroy_subscription(self._hand_depth_subscription)
        self.node.destroy_subscription(
            self._hand_depth_camera_info_subscription
        )
        self.node.destroy_subscription(
            self._end_effector_force_subscription
        )
        with self._lock:
            self._base_tag_histories.clear()
            self._hand_depth_history.clear()
            self._end_effector_force_history.clear()
            self._hand_depth_camera_info = None


__all__ = [
    "ProbeSetupMotionStateSource",
]
