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
from sensor_msgs.msg import CameraInfo, Image
import tf2_ros

from fault_detector_spot.inspection.model.models import (
    PoseData,
    QuaternionData,
    Vector3Data,
)
from fault_detector_spot.inspection.model.sensor_models import (
    sensor_probe_frame,
)
from fault_detector_spot.inspection.sensing.live_surface_distance import (
    measure_probe_surface_distance,
)
from fault_detector_spot.inspection.setup.reference_probe_setup import (
    compose_poses,
    relative_pose,
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

        now = self.node.get_clock().now()
        samples = []
        errors = []
        for receipt_time, depth_image in history:
            if receipt_time + 1e-9 < receipt_not_before:
                continue
            try:
                stamp = Time.from_msg(depth_image.header.stamp)
                if stamp.nanoseconds <= 0:
                    raise ValueError("Registered hand-depth timestamp is empty")
                age_seconds = (now - stamp).nanoseconds * 1e-9
                if age_seconds < -0.05:
                    raise ValueError(
                        "Registered hand-depth timestamp is in the future"
                    )
                if age_seconds > maximum_age_sec:
                    continue
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
            raise ValueError(
                "Need at least three fresh registered hand-depth samples: "
                f"{detail}"
            )
        return tuple(samples)

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

    def close(self) -> None:
        """Destroy ROS resources owned by this state source."""
        self.node.destroy_subscription(self._base_tag_subscription)
        self.node.destroy_subscription(self._hand_depth_subscription)
        self.node.destroy_subscription(
            self._hand_depth_camera_info_subscription
        )
        with self._lock:
            self._base_tag_histories.clear()
            self._hand_depth_history.clear()
            self._hand_depth_camera_info = None


__all__ = ["ProbeSetupMotionStateSource"]
