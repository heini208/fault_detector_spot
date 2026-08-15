"""Provide authoritative runtime state for probe setup movement."""

from collections import deque
from copy import deepcopy
from dataclasses import dataclass
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
from fault_detector_spot.inspection.sensing.live_surface_distance import (
    measure_probe_surface_distance,
)
from fault_detector_spot.inspection.setup.reference_probe_setup import (
    compose_poses,
    multiply_quaternions,
    relative_pose,
    rotate_vector,
)
from fault_detector_spot.inspection.setup.alignment_orientation import (
    surface_aligned_probe_orientation,
)
from fault_detector_spot.inspection.setup.reference_view_depth_projection import (
    project_reference_pixel,
)
from fault_detector_spot.inspection.setup.reference_view_surface_normal import (
    estimate_reference_surface_normal,
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
MINIMUM_SURFACE_DISTANCE_SAMPLES = 3
HAND_SURFACE_WINDOW_PARAMETER = "inspection.hand_surface_window_radius_px"
DEFAULT_HAND_SURFACE_WINDOW_RADIUS_PX = 16
MINIMUM_HAND_SURFACE_WINDOW_RADIUS_PX = 4
MAXIMUM_HAND_SURFACE_WINDOW_RADIUS_PX = 64
MINIMUM_HAND_CAMERA_SURFACE_CLEARANCE_M = 0.260
END_EFFECTOR_FORCE_TOPIC = "/status/end_effector_force"
END_EFFECTOR_FORCE_HISTORY_MAX_SAMPLES = 128
MAX_END_EFFECTOR_FORCE_AGE_SEC = 0.25


@dataclass(frozen=True)
class LiveHandSurfaceOrientation:
    """One current hand-camera surface orientation calculation."""

    probe_orientation_object: QuaternionData
    hand_orientation_object: QuaternionData
    surface_normal_object: Vector3Data
    sample_count: int
    plane_rmse_m: float


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
        node.declare_parameter(
            HAND_SURFACE_WINDOW_PARAMETER,
            DEFAULT_HAND_SURFACE_WINDOW_RADIUS_PX,
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
        for stamp_key, tag in history:
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
        return compose_poses(
            gravity_to_source,
            source_to_object,
        )

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

    def live_hand_surface_orientation(
        self,
        reference_tag_id: int,
        hand_to_probe_orientation: QuaternionData,
    ) -> LiveHandSurfaceOrientation:
        """Calculate a surface-facing probe orientation at hand-image center."""
        hand_to_probe_orientation.validate()
        with self._lock:
            camera_info = deepcopy(self._hand_depth_camera_info)
            history = tuple(self._hand_depth_history)
        if camera_info is None:
            raise ValueError(
                "No registered hand-depth camera info is available"
            )
        depth_image = None
        stamp = None
        fresh_history = self._recent_hand_depth_samples(
            history,
            MAX_HAND_DEPTH_AGE_SEC,
        )
        for _receipt_time, candidate in reversed(fresh_history):
            candidate_stamp = Time.from_msg(candidate.header.stamp)
            if candidate_stamp.nanoseconds <= 0:
                continue
            depth_image = deepcopy(candidate)
            stamp = candidate_stamp
            break
        if depth_image is None or stamp is None:
            raise ValueError(
                "No fresh registered hand-depth image is available"
            )
        window_radius_px = self._hand_surface_window_radius_px()
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
                rgb_size=(int(depth_image.width), int(depth_image.height)),
            )
        except ValueError as exception:
            if "No valid depth within" not in str(exception):
                raise
            raise ValueError(
                "No valid registered hand depth in the "
                f"{window_radius_px} px center window. The gripper ToF "
                "may be inside its usable near field. Increase the aligned "
                "pre-approach distance or the center-window radius."
            ) from exception
        normal = estimate_reference_surface_normal(
            projected,
            depth_image,
            camera_info,
            neighborhood_radius_px=window_radius_px,
            maximum_neighborhood_radius_px=window_radius_px,
        )
        tag = self.reference_tag(reference_tag_id)
        body_frame = tag.pose.header.frame_id.strip()
        body_to_object = pose_to_pose_data(tag.pose.pose)
        body_to_camera = self._lookup_pose(
            body_frame,
            projected.frame_id,
            lookup_time=stamp,
        )
        object_to_camera = relative_pose(
            body_to_object,
            body_to_camera,
        )
        surface_normal_object = rotate_vector(
            object_to_camera.orientation,
            normal.normal_camera,
        )
        gravity_to_object = self.gravity_aligned_object_pose(
            reference_tag_id
        )
        gravity_up_object = rotate_vector(
            self._inverse_orientation(gravity_to_object.orientation),
            Vector3Data(x=0.0, y=0.0, z=1.0),
        )
        probe_orientation = surface_aligned_probe_orientation(
            surface_normal_object,
            hand_to_probe_orientation,
            gravity_up_object,
        )
        hand_orientation = multiply_quaternions(
            probe_orientation,
            self._inverse_orientation(hand_to_probe_orientation),
        )
        return LiveHandSurfaceOrientation(
            probe_orientation_object=probe_orientation,
            hand_orientation_object=hand_orientation,
            surface_normal_object=surface_normal_object,
            sample_count=normal.sample_count,
            plane_rmse_m=normal.plane_rmse_m,
        )

    def _hand_surface_window_radius_px(self) -> int:
        value = self.node.get_parameter(
            HAND_SURFACE_WINDOW_PARAMETER
        ).value
        if isinstance(value, bool) or not isinstance(value, int):
            raise ValueError(
                "Hand surface window radius must be an integer"
            )
        if not (
            MINIMUM_HAND_SURFACE_WINDOW_RADIUS_PX
            <= value
            <= MAXIMUM_HAND_SURFACE_WINDOW_RADIUS_PX
        ):
            raise ValueError(
                "Hand surface window radius must be between "
                f"{MINIMUM_HAND_SURFACE_WINDOW_RADIUS_PX} and "
                f"{MAXIMUM_HAND_SURFACE_WINDOW_RADIUS_PX} pixels"
            )
        return value

    @staticmethod
    def _inverse_orientation(orientation: QuaternionData) -> QuaternionData:
        orientation.validate()
        return QuaternionData(
            x=-orientation.x,
            y=-orientation.y,
            z=-orientation.z,
            w=orientation.w,
        )

    def minimum_aligned_probe_distance_m(
        self,
        sensor_id: str,
        minimum_camera_clearance_m: float = (
            MINIMUM_HAND_CAMERA_SURFACE_CLEARANCE_M
        ),
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
        minimum_camera_clearance_m: float = (
            MINIMUM_HAND_CAMERA_SURFACE_CLEARANCE_M
        ),
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
            search_radius_px=self._hand_surface_window_radius_px(),
            rgb_size=(int(depth_image.width), int(depth_image.height)),
        )
        return float(projected.depth_m)

    def require_hand_camera_clearance(
        self,
        minimum_camera_clearance_m: float = (
            MINIMUM_HAND_CAMERA_SURFACE_CLEARANCE_M
        ),
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

    def surface_distance_samples(
        self,
        sensor_id: str,
        receipt_not_before: float = 0.0,
        maximum_age_sec: float = MAX_HAND_DEPTH_AGE_SEC,
    ):
        """Return fresh registered-depth measurements along the probe axis."""
        if not isinstance(sensor_id, str) or not sensor_id.strip():
            raise ValueError("Sensor ID must not be empty")
        if not math.isfinite(float(receipt_not_before)):
            raise ValueError("Surface sample receipt threshold must be finite")
        if (
            not math.isfinite(float(maximum_age_sec))
            or maximum_age_sec <= 0.0
        ):
            raise ValueError("Maximum hand-depth age must be positive")
        with self._lock:
            camera_info = deepcopy(self._hand_depth_camera_info)
            history = tuple(self._hand_depth_history)
        if camera_info is None:
            raise ValueError(
                "No registered hand-depth camera info is available"
            )

        samples = []
        errors = []
        fresh_history = self._recent_hand_depth_samples(
            history,
            maximum_age_sec,
            receipt_not_before=receipt_not_before,
        )
        for _receipt_time, depth_image in fresh_history:
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
                    sensor_probe_frame(sensor_id.strip()),
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
        if len(samples) < MINIMUM_SURFACE_DISTANCE_SAMPLES:
            detail = errors[-1] if errors else "no fresh depth frames"
            if (
                "no valid pixels" in detail.lower()
                or "insufficient support" in detail.lower()
            ):
                detail = (
                    f"{detail}. The gripper ToF may be inside its usable "
                    "near field at this probe distance"
                )
            raise ValueError(
                "Need at least three fresh registered hand-depth samples: "
                f"{detail}"
            )
        return tuple(samples)

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
            stamp_seconds = (
                float(stamp.sec) + float(stamp.nanosec) * 1e-9
            )
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
        with self._lock:
            for tag in message.elements:
                stamp = tag.pose.header.stamp
                stamp_key = (int(stamp.sec), int(stamp.nanosec))
                history = self._base_tag_histories.setdefault(
                    int(tag.id),
                    deque(maxlen=BASE_TAG_HISTORY_MAX_SAMPLES),
                )
                if history and history[-1][0] == stamp_key:
                    continue
                history.append((stamp_key, deepcopy(tag)))

    def _receive_hand_depth(self, message: Image) -> None:
        with self._lock:
            self._hand_depth_history.append(
                (time.monotonic(), deepcopy(message))
            )

    def _receive_hand_depth_camera_info(
        self,
        message: CameraInfo,
    ) -> None:
        with self._lock:
            self._hand_depth_camera_info = deepcopy(message)

    def _receive_end_effector_force(
        self,
        message: Vector3Stamped,
    ) -> None:
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
    "LiveHandSurfaceOrientation",
    "ProbeSetupMotionStateSource",
]
