"""Read Spot's measured body pose and planar projection from one TF sample."""

from dataclasses import dataclass
import math

from bosdyn.client.frame_helpers import BODY_FRAME_NAME, ODOM_FRAME_NAME

from fault_detector_spot.shared.geometry.rotation import (
    quaternion_to_rpy,
)
from fault_detector_spot.shared.geometry.models import (
    PoseData,
)
from fault_detector_spot.shared.ros.tf_transforms import transform_to_pose_data


@dataclass(frozen=True)
class BasePoseSample:
    """Planar feedback and optional full body geometry at the same timestamp."""

    x_m: float
    y_m: float
    yaw_rad: float
    stamp_sec: float
    body_pose: PoseData | None = None

    @property
    def planar_pose(self):
        return (self.x_m, self.y_m, self.yaw_rad)


class BasePoseSource:
    """Measure the latest odom-to-body base pose from shared TF."""

    def __init__(self, tf_listener):
        if tf_listener is None:
            raise RuntimeError("BasePoseSource requires a TF listener")
        self.tf_listener = tf_listener

    def sample(self) -> BasePoseSample | None:
        try:
            transform = self.tf_listener.lookup_a_tform_b(
                ODOM_FRAME_NAME,
                BODY_FRAME_NAME,
                timeout_sec=0.0,
            )
            body_pose = transform_to_pose_data(transform)
            translation = body_pose.position
            _, _, yaw = quaternion_to_rpy(body_pose.orientation)
            stamp = (
                float(transform.header.stamp.sec)
                + float(transform.header.stamp.nanosec) * 1e-9
            )
            values = (
                float(translation.x),
                float(translation.y),
                float(yaw),
                stamp,
            )
        except Exception:
            return None

        if not all(math.isfinite(value) for value in values):
            return None

        return BasePoseSample(
            x_m=values[0],
            y_m=values[1],
            yaw_rad=values[2],
            stamp_sec=values[3],
            body_pose=body_pose,
        )


__all__ = ["BasePoseSample", "BasePoseSource"]
