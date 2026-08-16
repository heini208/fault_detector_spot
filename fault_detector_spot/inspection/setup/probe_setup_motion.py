"""Define and translate single-step probe setup movements."""

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

from bosdyn.client.frame_helpers import HAND_FRAME_NAME
from fault_detector_msgs.msg import TagElement

from fault_detector_spot.application.commanding.command_ids import (
    CommandID,
    OrientationModes,
)
from fault_detector_spot.application.commanding.semantic_command import (
    CommandQuaternion,
    CommandVector3,
    SemanticCommand,
    SemanticTag,
    StampedPose,
)
from fault_detector_spot.inspection.model.models import (
    PoseData,
    QuaternionData,
    Vector3Data,
)
from fault_detector_spot.inspection.model.sensor_models import (
    sensor_probe_frame,
)
from fault_detector_spot.inspection.setup.reference_probe_setup import (
    compose_poses,
)
from fault_detector_spot.shared.geometry.transforms import pose_to_pose_data


MAX_REFINEMENT_TRANSLATION_M = 0.05
MAX_REFINEMENT_ROTATION_RAD = math.radians(15.0)


class ProbeMotionKind(str, Enum):
    """Supported single-step setup movement kinds."""

    MOVE_SAFE_APPROACH = "move_safe_approach"
    MOVE_ALIGNED_PREAPPROACH = "move_aligned_preapproach"
    ADJUST_SAFE_APPROACH = "adjust_safe_approach"
    ADJUST_ALIGNED_PREAPPROACH = "adjust_aligned_preapproach"
    ADJUST_PROBE_DISTANCE = "adjust_probe_distance"


class ProbeMotionFrame(str, Enum):
    """Public frame selections accepted for relative refinement."""

    SENSOR = "sensor"
    HAND = "hand"
    TAG = "tag"
    BODY = "body"
    MAP = "map"


class ProbeAlignmentOrientationMode(str, Enum):
    """Explicit orientation source for one aligned absolute move."""

    TAG = "tag"
    CALCULATED_SURFACE = "calculated_surface"


@dataclass(frozen=True)
class ProbeMotionRequest:
    """Describe one validated setup motion primitive."""

    kind: ProbeMotionKind
    frame: ProbeMotionFrame = ProbeMotionFrame.SENSOR
    translation: Vector3Data = field(default_factory=Vector3Data.zero)
    pitch_rad: float = 0.0
    yaw_rad: float = 0.0
    position_tolerance_m: float = 0.01
    orientation_tolerance_rad: float = math.radians(5.0)
    alignment_orientation_mode: ProbeAlignmentOrientationMode = (
        ProbeAlignmentOrientationMode.TAG
    )
    orientation_only: bool = False
    calculated_surface_orientation_object: Optional[
        QuaternionData
    ] = None

    def validate(self) -> None:
        if not isinstance(self.kind, ProbeMotionKind):
            raise TypeError("Probe motion kind is invalid")
        if not isinstance(self.frame, ProbeMotionFrame):
            raise TypeError("Probe motion frame is invalid")
        if not isinstance(
            self.alignment_orientation_mode,
            ProbeAlignmentOrientationMode,
        ):
            raise TypeError("Alignment orientation mode is invalid")
        if not isinstance(self.orientation_only, bool):
            raise TypeError("Orientation-only flag must be boolean")
        self.translation.validate()
        for value, label in (
            (self.pitch_rad, "Pitch adjustment"),
            (self.yaw_rad, "Yaw adjustment"),
            (self.position_tolerance_m, "Position tolerance"),
            (self.orientation_tolerance_rad, "Orientation tolerance"),
        ):
            if not math.isfinite(float(value)):
                raise ValueError(f"{label} must be finite")
        if any(
            abs(value) > MAX_REFINEMENT_TRANSLATION_M
            for value in (
                self.translation.x,
                self.translation.y,
                self.translation.z,
            )
        ):
            raise ValueError("Refinement translation exceeds 0.05 m")
        if abs(self.pitch_rad) > MAX_REFINEMENT_ROTATION_RAD:
            raise ValueError("Pitch adjustment exceeds 15 degrees")
        if abs(self.yaw_rad) > MAX_REFINEMENT_ROTATION_RAD:
            raise ValueError("Yaw adjustment exceeds 15 degrees")
        if self.position_tolerance_m <= 0.0:
            raise ValueError("Position tolerance must be positive")
        if self.orientation_tolerance_rad <= 0.0:
            raise ValueError("Orientation tolerance must be positive")
        if (
            self.orientation_only
            and self.kind is not ProbeMotionKind.MOVE_ALIGNED_PREAPPROACH
        ):
            raise ValueError(
                "Orientation-only motion is available only for alignment"
            )
        if (
            self.alignment_orientation_mode
            is ProbeAlignmentOrientationMode.CALCULATED_SURFACE
        ):
            if self.kind is not ProbeMotionKind.MOVE_ALIGNED_PREAPPROACH:
                raise ValueError(
                    "Calculated surface orientation is available only "
                    "for alignment"
                )
            if self.calculated_surface_orientation_object is None:
                raise ValueError(
                    "Calculated surface alignment requires an orientation"
                )
            self.calculated_surface_orientation_object.validate()
        elif self.calculated_surface_orientation_object is not None:
            self.calculated_surface_orientation_object.validate()

    @property
    def relative(self) -> bool:
        return self.kind in {
            ProbeMotionKind.ADJUST_SAFE_APPROACH,
            ProbeMotionKind.ADJUST_ALIGNED_PREAPPROACH,
            ProbeMotionKind.ADJUST_PROBE_DISTANCE,
        }


class ProbeSetupMotionCommandFactory:
    """Translate one probe setup movement into one semantic command."""

    def absolute(
        self,
        target_probe_pose_object: PoseData,
        reference_tag: TagElement,
        motion_sensor_id: str,
    ) -> SemanticCommand:
        target_probe_pose_object.validate()
        sensor_id = self._required_sensor_id(motion_sensor_id)

        frame_id = reference_tag.pose.header.frame_id.strip()
        if not frame_id:
            raise ValueError("Reference tag pose frame is empty")

        body_to_object = pose_to_pose_data(reference_tag.pose.pose)
        body_to_probe = compose_poses(
            body_to_object,
            target_probe_pose_object,
        )

        tag_pose = StampedPose(
            frame_id=reference_tag.pose.header.frame_id,
            stamp_sec=int(reference_tag.pose.header.stamp.sec),
            stamp_nanosec=int(reference_tag.pose.header.stamp.nanosec),
            position=CommandVector3(
                x=reference_tag.pose.pose.position.x,
                y=reference_tag.pose.pose.position.y,
                z=reference_tag.pose.pose.position.z,
            ),
            orientation=CommandQuaternion(
                x=reference_tag.pose.pose.orientation.x,
                y=reference_tag.pose.pose.orientation.y,
                z=reference_tag.pose.pose.orientation.z,
                w=reference_tag.pose.pose.orientation.w,
            ),
        )

        offset = StampedPose(
            frame_id=frame_id,
            stamp_sec=int(reference_tag.pose.header.stamp.sec),
            stamp_nanosec=int(reference_tag.pose.header.stamp.nanosec),
            position=CommandVector3(
                x=body_to_probe.position.x - body_to_object.position.x,
                y=body_to_probe.position.y - body_to_object.position.y,
                z=body_to_probe.position.z - body_to_object.position.z,
            ),
            orientation=CommandQuaternion(
                x=body_to_probe.orientation.x,
                y=body_to_probe.orientation.y,
                z=body_to_probe.orientation.z,
                w=body_to_probe.orientation.w,
            ),
        )

        return SemanticCommand(
            command_id=CommandID.MOVE_ARM_TO_TAG,
            tag=SemanticTag(
                id=int(reference_tag.id),
                pose=tag_pose,
            ),
            offset=offset,
            orientation_mode=OrientationModes.CUSTOM_ORIENTATION.value,
            motion_sensor_id=sensor_id,
        )

    def relative(
        self,
        frame_id: str,
        translation: Vector3Data,
        pitch_rad: float,
        yaw_rad: float,
        motion_sensor_id: str,
    ) -> SemanticCommand:
        if not isinstance(frame_id, str) or not frame_id.strip():
            raise ValueError("Refinement frame must not be empty")
        sensor_id = self._required_sensor_id(motion_sensor_id)

        translation.validate()
        rotation = self._relative_rotation(pitch_rad, yaw_rad)

        return SemanticCommand(
            command_id=CommandID.MOVE_ARM_RELATIVE,
            offset=StampedPose(
                frame_id=frame_id.strip(),
                position=CommandVector3(
                    x=translation.x,
                    y=translation.y,
                    z=translation.z,
                ),
                orientation=CommandQuaternion(
                    x=rotation.x,
                    y=rotation.y,
                    z=rotation.z,
                    w=rotation.w,
                ),
            ),
            motion_sensor_id=sensor_id,
        )

    @staticmethod
    def frame_id(
        frame: ProbeMotionFrame,
        sensor_id: str,
        reference_tag_id: int,
    ) -> str:
        values = {
            ProbeMotionFrame.SENSOR: sensor_probe_frame(sensor_id),
            ProbeMotionFrame.HAND: HAND_FRAME_NAME,
            ProbeMotionFrame.TAG: f"filtered_fiducial_{reference_tag_id}",
            ProbeMotionFrame.BODY: "body",
            ProbeMotionFrame.MAP: "map",
        }
        return values[frame]

    @staticmethod
    def _required_sensor_id(sensor_id: str) -> str:
        if not isinstance(sensor_id, str):
            raise TypeError("Motion sensor ID must be a string")
        normalized = sensor_id.strip()
        if not normalized:
            raise ValueError("Motion sensor ID must not be empty")
        return normalized

    @staticmethod
    def _relative_rotation(pitch_rad, yaw_rad):
        half_pitch = pitch_rad * 0.5
        half_yaw = yaw_rad * 0.5
        return QuaternionData(
            x=-math.sin(half_pitch) * math.sin(half_yaw),
            y=math.sin(half_pitch) * math.cos(half_yaw),
            z=math.cos(half_pitch) * math.sin(half_yaw),
            w=math.cos(half_pitch) * math.cos(half_yaw),
        )


__all__ = [
    "ProbeAlignmentOrientationMode",
    "ProbeMotionFrame",
    "ProbeMotionKind",
    "ProbeMotionRequest",
    "ProbeSetupMotionCommandFactory",
]
