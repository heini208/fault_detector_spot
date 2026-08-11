"""Provide authoritative runtime state for probe setup movement."""

from collections import deque
from copy import deepcopy
from threading import RLock

from fault_detector_msgs.msg import TagElementArray
from rclpy.duration import Duration
from rclpy.time import Time
import tf2_ros

from fault_detector_spot.inspection.model.models import (
    PoseData,
    QuaternionData,
    Vector3Data,
)
from fault_detector_spot.inspection.model.sensor_models import (
    sensor_probe_frame,
)
from fault_detector_spot.inspection.setup.reference_probe_setup import (
    relative_pose,
)
from fault_detector_spot.inspection.setup.stable_tag_pose import (
    TagPoseSample,
    stabilize_tag_pose,
)
from fault_detector_spot.shared.geometry.transforms import pose_to_pose_data


BASE_TAG_MAXIMUM_AGE_SEC = 1.5
BASE_TAG_STABILIZATION_HISTORY_SEC = 3.0
BASE_TAG_HISTORY_MAX_SAMPLES = 64
BASE_TAG_MINIMUM_SPAN_SEC = 0.10


class ProbeSetupMotionStateSource:
    """Resolve stable tag and probe poses outside the remote UI."""

    def __init__(self, node):
        self.node = node
        self._lock = RLock()
        self._base_tag_histories = {}
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

    def _lookup_pose(self, target_frame: str, source_frame: str) -> PoseData:
        transform = self._tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            Time(),
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

    def close(self) -> None:
        """Destroy ROS resources owned by this state source."""
        self.node.destroy_subscription(self._base_tag_subscription)
        with self._lock:
            self._base_tag_histories.clear()


__all__ = ["ProbeSetupMotionStateSource"]
