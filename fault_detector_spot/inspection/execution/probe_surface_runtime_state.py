"""Live robot state required by the standalone close-surface command."""

from collections import deque
from copy import deepcopy
from threading import RLock
import math
import time

from bosdyn.client.frame_helpers import (
    GRAV_ALIGNED_BODY_FRAME_NAME,
    HAND_FRAME_NAME,
)
from fault_detector_msgs.msg import SensorAttachmentState
from geometry_msgs.msg import Vector3Stamped
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
    BARE_HAND_MOTION_ID,
    sensor_probe_frame,
)
from fault_detector_spot.inspection.sensing.end_effector_force import (
    EndEffectorForceSample,
)
from fault_detector_spot.inspection.sensing.live_surface_distance import (
    measure_probe_surface_distance,
)
from fault_detector_spot.inspection.setup.reference_probe_setup import (
    rotate_vector,
)
from fault_detector_spot.shared.ros.qos_profiles import APPLICATION_STATE_QOS


HAND_DEPTH_HISTORY_MAX_SAMPLES = 32
END_EFFECTOR_FORCE_HISTORY_MAX_SAMPLES = 128
MAX_HAND_DEPTH_AGE_SEC = 0.5
MAX_END_EFFECTOR_FORCE_AGE_SEC = 0.25
SENSOR_ATTACHMENT_TOPIC = "fault_detector/application/sensor_attachment_state"
END_EFFECTOR_FORCE_TOPIC = "/status/end_effector_force"


class ProbeSurfaceRuntimeStateSource:
    """Expose depth, force, attachment, and TF state without setup context."""

    def __init__(self, node):
        self.node = node
        self._lock = RLock()
        self._hand_depth_history = deque(
            maxlen=HAND_DEPTH_HISTORY_MAX_SAMPLES
        )
        self._hand_depth_camera_info = None
        self._end_effector_force_history = deque(
            maxlen=END_EFFECTOR_FORCE_HISTORY_MAX_SAMPLES
        )
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
        self._force_subscription = node.create_subscription(
            Vector3Stamped,
            END_EFFECTOR_FORCE_TOPIC,
            self._receive_end_effector_force,
            qos_profile_sensor_data,
        )
        self._attachment_subscription = node.create_subscription(
            SensorAttachmentState,
            SENSOR_ATTACHMENT_TOPIC,
            self._receive_attachment_state,
            APPLICATION_STATE_QOS,
        )

    def active_attachment(self):
        """Return the effective motion sensor and frozen attachment revision."""
        with self._lock:
            state = deepcopy(self._attachment_state)
        if state is None:
            raise ValueError("Sensor attachment state is unavailable")
        if state.status == SensorAttachmentState.STATUS_CONFIRMATION_PENDING:
            raise ValueError("Sensor attachment confirmation is pending")
        if state.status == SensorAttachmentState.STATUS_ACTIVE:
            sensor_id = state.active_sensor_id.strip()
            if not sensor_id:
                raise ValueError("Active sensor attachment has no sensor ID")
        elif state.status == SensorAttachmentState.STATUS_NO_SENSOR:
            sensor_id = BARE_HAND_MOTION_ID
        else:
            raise ValueError("Sensor attachment state is invalid")
        return sensor_id, int(state.attachment_revision)

    def current_probe_pose_execution(self, sensor_id: str) -> PoseData:
        """Return current probe pose in Spot's gravity-aligned body frame."""
        return self._lookup_pose(
            GRAV_ALIGNED_BODY_FRAME_NAME,
            sensor_probe_frame(sensor_id),
        )

    def current_hand_pose_execution(self) -> PoseData:
        """Return current Spot hand pose in the gravity-aligned body frame."""
        return self._lookup_pose(
            GRAV_ALIGNED_BODY_FRAME_NAME,
            HAND_FRAME_NAME,
        )

    def probe_axis_hand(self, sensor_id: str) -> Vector3Data:
        """Return probe local positive X expressed in the hand frame."""
        if sensor_probe_frame(sensor_id) == HAND_FRAME_NAME:
            return Vector3Data(x=1.0, y=0.0, z=0.0)
        hand_to_probe = self._lookup_pose(
            HAND_FRAME_NAME,
            sensor_probe_frame(sensor_id),
        )
        return rotate_vector(
            hand_to_probe.orientation,
            Vector3Data(x=1.0, y=0.0, z=0.0),
        )

    def surface_distance_samples(
        self,
        sensor_id: str,
        receipt_not_before: float = 0.0,
        maximum_age_sec: float = MAX_HAND_DEPTH_AGE_SEC,
        minimum_samples: int = 1,
    ):
        """Return fresh probe-axis surface-distance samples."""
        if not sensor_id.strip():
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
        for receipt_time, depth_image in history:
            if receipt_time + 1e-9 < receipt_not_before:
                continue
            age = now - receipt_time
            if age < -1e-9 or age > maximum_age_sec:
                continue
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
            detail = errors[-1] if errors else "no fresh depth frames"
            raise ValueError(
                "Need at least "
                f"{minimum_samples} fresh registered hand-depth sample"
                f"{'s' if minimum_samples != 1 else ''}: {detail}"
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
        if not math.isfinite(float(maximum_age_sec)) or maximum_age_sec <= 0.0:
            raise ValueError(
                "Maximum end-effector force age must be positive"
            )
        with self._lock:
            history = tuple(self._end_effector_force_history)
        now = time.monotonic()
        samples = []
        for receipt_time, message in history:
            if receipt_time + 1e-9 < receipt_not_before:
                continue
            age = now - receipt_time
            if age < -1e-9 or age > maximum_age_sec:
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
            maximum_age_sec=maximum_age_sec,
        )[-1]

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

    def _receive_attachment_state(self, message: SensorAttachmentState) -> None:
        with self._lock:
            self._attachment_state = deepcopy(message)

    def close(self) -> None:
        """Destroy ROS resources owned by this source."""
        self.node.destroy_subscription(self._hand_depth_subscription)
        self.node.destroy_subscription(
            self._hand_depth_camera_info_subscription
        )
        self.node.destroy_subscription(self._force_subscription)
        self.node.destroy_subscription(self._attachment_subscription)
        with self._lock:
            self._hand_depth_history.clear()
            self._end_effector_force_history.clear()
            self._hand_depth_camera_info = None
            self._attachment_state = None


__all__ = ["ProbeSurfaceRuntimeStateSource"]
