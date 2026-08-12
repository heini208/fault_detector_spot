"""ROS-independent semantic robot command model."""

import math
from dataclasses import dataclass, field

from fault_detector_spot.application.commanding.command_ids import CommandID


def _finite(value, label):
    normalized = float(value)
    if not math.isfinite(normalized):
        raise ValueError(f"{label} must be finite")
    return normalized


@dataclass(frozen=True)
class CommandVector3:
    """Plain three-dimensional command vector."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def __post_init__(self):
        object.__setattr__(self, "x", _finite(self.x, "Vector x"))
        object.__setattr__(self, "y", _finite(self.y, "Vector y"))
        object.__setattr__(self, "z", _finite(self.z, "Vector z"))


@dataclass(frozen=True)
class CommandQuaternion:
    """Plain command quaternion without ROS ownership."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 0.0

    def __post_init__(self):
        object.__setattr__(self, "x", _finite(self.x, "Quaternion x"))
        object.__setattr__(self, "y", _finite(self.y, "Quaternion y"))
        object.__setattr__(self, "z", _finite(self.z, "Quaternion z"))
        object.__setattr__(self, "w", _finite(self.w, "Quaternion w"))


@dataclass(frozen=True)
class StampedPose:
    """Plain stamped pose used by semantic commands."""

    frame_id: str = ""
    stamp_sec: int = 0
    stamp_nanosec: int = 0
    position: CommandVector3 = field(default_factory=CommandVector3)
    orientation: CommandQuaternion = field(
        default_factory=CommandQuaternion
    )

    def __post_init__(self):
        if not isinstance(self.frame_id, str):
            raise TypeError("Pose frame ID must be a string")
        if isinstance(self.stamp_sec, bool) or not isinstance(
            self.stamp_sec, int
        ):
            raise TypeError("Pose stamp seconds must be an integer")
        if isinstance(self.stamp_nanosec, bool) or not isinstance(
            self.stamp_nanosec, int
        ):
            raise TypeError("Pose stamp nanoseconds must be an integer")
        if self.stamp_nanosec < 0 or self.stamp_nanosec >= 1_000_000_000:
            raise ValueError(
                "Pose stamp nanoseconds must be in [0, 1000000000)"
            )
        if not isinstance(self.position, CommandVector3):
            raise TypeError("Pose position must be CommandVector3")
        if not isinstance(self.orientation, CommandQuaternion):
            raise TypeError(
                "Pose orientation must be CommandQuaternion"
            )
        object.__setattr__(self, "frame_id", self.frame_id.strip())


@dataclass(frozen=True)
class SemanticTag:
    """Plain AprilTag selection and observed pose."""

    id: int
    pose: StampedPose

    def __post_init__(self):
        if isinstance(self.id, bool) or not isinstance(self.id, int):
            raise TypeError("Tag ID must be an integer")
        if self.id < 0:
            raise ValueError("Tag ID must not be negative")
        if not isinstance(self.pose, StampedPose):
            raise TypeError("Tag pose must be a StampedPose")


@dataclass(frozen=True)
class InspectionSelection:
    """Persisted inspection target selected by a semantic command."""

    object_id: str = ""
    routine_id: str = ""
    probe_point_id: str = ""

    def __post_init__(self):
        for name, value in (
            ("object_id", self.object_id),
            ("routine_id", self.routine_id),
            ("probe_point_id", self.probe_point_id),
        ):
            if not isinstance(value, str):
                raise TypeError(f"{name} must be a string")
            object.__setattr__(self, name, value.strip())


@dataclass(frozen=True)
class SemanticCommand:
    """Application-owned robot command independent of ROS messages."""

    command_id: CommandID
    tag: SemanticTag | None = None
    offset: StampedPose = field(default_factory=StampedPose)
    orientation_mode: str = ""
    wait_time: float = 0.0
    map_name: str = ""
    waypoint_name: str = ""
    inspection: InspectionSelection = field(
        default_factory=InspectionSelection
    )

    def __post_init__(self):
        try:
            command_id = CommandID(self.command_id)
        except (TypeError, ValueError) as exception:
            raise ValueError(
                f"Unsupported command ID: {self.command_id!r}"
            ) from exception
        if self.tag is not None and not isinstance(
            self.tag, SemanticTag
        ):
            raise TypeError("Command tag must be SemanticTag or None")
        if not isinstance(self.offset, StampedPose):
            raise TypeError("Command offset must be StampedPose")
        if not isinstance(self.orientation_mode, str):
            raise TypeError("Orientation mode must be a string")
        if not isinstance(self.map_name, str):
            raise TypeError("Map name must be a string")
        if not isinstance(self.waypoint_name, str):
            raise TypeError("Waypoint name must be a string")
        if not isinstance(self.inspection, InspectionSelection):
            raise TypeError(
                "Inspection selection must be InspectionSelection"
            )
        object.__setattr__(self, "command_id", command_id)
        object.__setattr__(
            self,
            "orientation_mode",
            self.orientation_mode.strip(),
        )
        object.__setattr__(self, "wait_time", _finite(
            self.wait_time,
            "Wait time",
        ))
        object.__setattr__(self, "map_name", self.map_name.strip())
        object.__setattr__(
            self,
            "waypoint_name",
            self.waypoint_name.strip(),
        )


__all__ = [
    "CommandQuaternion",
    "CommandVector3",
    "InspectionSelection",
    "SemanticCommand",
    "SemanticTag",
    "StampedPose",
]
