"""Define and translate single-step probe setup movements."""

import math
from copy import deepcopy
from dataclasses import dataclass, field
from enum import Enum

from bosdyn.client.frame_helpers import HAND_FRAME_NAME
from fault_detector_msgs.msg import ComplexCommand, TagElement

from fault_detector_spot.application.commanding.command_ids import (
    CommandID,
    OrientationModes,
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
    probe_pose_to_hand_pose,
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

    def validate(self) -> None:
        if not isinstance(self.kind, ProbeMotionKind):
            raise TypeError("Probe motion kind is invalid")
        if not isinstance(self.frame, ProbeMotionFrame):
            raise TypeError("Probe motion frame is invalid")
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

    @property
    def relative(self) -> bool:
        return self.kind in {
            ProbeMotionKind.ADJUST_SAFE_APPROACH,
            ProbeMotionKind.ADJUST_ALIGNED_PREAPPROACH,
            ProbeMotionKind.ADJUST_PROBE_DISTANCE,
        }


class ProbeSetupMotionCommandFactory:
    """Translate one probe setup movement into one internal command."""

    def absolute(
        self,
        target_probe_pose_object: PoseData,
        hand_to_probe: PoseData,
        reference_tag: TagElement,
    ) -> ComplexCommand:
        target_probe_pose_object.validate()
        hand_to_probe.validate()
        frame_id = reference_tag.pose.header.frame_id.strip()
        if not frame_id:
            raise ValueError("Reference tag pose frame is empty")
        body_to_object = pose_to_pose_data(reference_tag.pose.pose)
        object_to_hand = probe_pose_to_hand_pose(
            target_probe_pose_object,
            hand_to_probe,
        )
        body_to_hand = compose_poses(body_to_object, object_to_hand)

        command = ComplexCommand()
        command.command.command_id = CommandID.MOVE_ARM_TO_TAG.value
        command.tag = deepcopy(reference_tag)
        command.offset.header = deepcopy(reference_tag.pose.header)
        command.offset.header.frame_id = frame_id
        command.offset.pose.position.x = (
            body_to_hand.position.x - body_to_object.position.x
        )
        command.offset.pose.position.y = (
            body_to_hand.position.y - body_to_object.position.y
        )
        command.offset.pose.position.z = (
            body_to_hand.position.z - body_to_object.position.z
        )
        self._write_quaternion(
            command.offset.pose.orientation,
            body_to_hand.orientation,
        )
        command.orientation_mode = (
            OrientationModes.CUSTOM_ORIENTATION.value
        )
        return command

    def relative(
        self,
        frame_id: str,
        translation: Vector3Data,
        pitch_rad: float,
        yaw_rad: float,
    ) -> ComplexCommand:
        if not isinstance(frame_id, str) or not frame_id.strip():
            raise ValueError("Refinement frame must not be empty")
        translation.validate()
        command = ComplexCommand()
        command.command.command_id = CommandID.MOVE_ARM_RELATIVE.value
        command.offset.header.frame_id = frame_id.strip()
        command.offset.pose.position.x = translation.x
        command.offset.pose.position.y = translation.y
        command.offset.pose.position.z = translation.z
        self._write_quaternion(
            command.offset.pose.orientation,
            self._relative_rotation(pitch_rad, yaw_rad),
        )
        return command

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
    def _relative_rotation(pitch_rad, yaw_rad):
        half_pitch = pitch_rad * 0.5
        half_yaw = yaw_rad * 0.5
        return QuaternionData(
            x=-math.sin(half_pitch) * math.sin(half_yaw),
            y=math.sin(half_pitch) * math.cos(half_yaw),
            z=math.cos(half_pitch) * math.sin(half_yaw),
            w=math.cos(half_pitch) * math.cos(half_yaw),
        )

    @staticmethod
    def _write_quaternion(message, value):
        message.x = value.x
        message.y = value.y
        message.z = value.z
        message.w = value.w


__all__ = [
    "ProbeMotionFrame",
    "ProbeMotionKind",
    "ProbeMotionRequest",
    "ProbeSetupMotionCommandFactory",
]
