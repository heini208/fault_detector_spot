"""Persistent definitions for hand-mounted inspection sensors."""

import math
import re
from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, Tuple

from .models import PoseData, QuaternionData, Vector3Data
from fault_detector_spot.shared.persistence.file_storage import (
    validate_storage_name,
)


SENSOR_PARENT_FRAME = "hand"
BARE_HAND_MOTION_ID = SENSOR_PARENT_FRAME
_SENSOR_ID_PATTERN = re.compile(r"^[a-z][a-z0-9_]*$")


class SensorChannelSource(str, Enum):
    """Supported origins for one recorded channel."""

    ROS_TOPIC = "ros_topic"
    SPOT_GEOMETRY = "spot_geometry"


def sensor_probe_frame(sensor_id: str) -> str:
    """Resolve the probe frame for a sensor or the bare hand."""
    validate_storage_name(sensor_id, "sensor ID")
    if sensor_id == BARE_HAND_MOTION_ID:
        return SENSOR_PARENT_FRAME
    if _SENSOR_ID_PATTERN.fullmatch(sensor_id) is None:
        raise ValueError(
            "Sensor ID must start with a lowercase letter and contain "
            "only lowercase letters, digits, and underscores"
        )
    return f"{sensor_id}_probe"


def quaternion_from_rpy_degrees(
    roll_degrees: float,
    pitch_degrees: float,
    yaw_degrees: float,
) -> QuaternionData:
    """Convert fixed-axis roll, pitch, and yaw degrees to a quaternion."""
    values = (roll_degrees, pitch_degrees, yaw_degrees)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("Sensor rotation contains a non-finite value")

    roll = math.radians(roll_degrees)
    pitch = math.radians(pitch_degrees)
    yaw = math.radians(yaw_degrees)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    result = QuaternionData(
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
        w=cr * cp * cy + sr * sp * sy,
    )
    result.validate()
    return result


def rpy_degrees_from_quaternion(
    quaternion: QuaternionData,
) -> Tuple[float, float, float]:
    """Convert a normalized quaternion to fixed-axis RPY degrees."""
    quaternion.validate()
    x = float(quaternion.x)
    y = float(quaternion.y)
    z = float(quaternion.z)
    w = float(quaternion.w)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    x /= norm
    y /= norm
    z /= norm
    w /= norm

    roll = math.atan2(
        2.0 * (w * x + y * z),
        1.0 - 2.0 * (x * x + y * y),
    )
    pitch_input = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, pitch_input)))
    yaw = math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )
    return (
        math.degrees(roll),
        math.degrees(pitch),
        math.degrees(yaw),
    )


@dataclass(frozen=True)
class SensorChannel:
    """One configured measurement stream on a sensor mount."""

    channel_id: str
    topic: str
    message_type: str
    source_kind: SensorChannelSource = SensorChannelSource.ROS_TOPIC

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "SensorChannel":
        """Create a channel from serialized YAML data."""
        if not isinstance(data, dict):
            raise ValueError("sensor channel must be an object")
        return cls(
            channel_id=str(data["channel_id"]),
            topic=str(data["topic"]),
            message_type=str(data["message_type"]),
            source_kind=SensorChannelSource(
                data.get("source_kind", SensorChannelSource.ROS_TOPIC.value)
            ),
        )

    def validate(self) -> None:
        """Validate persistent identity and source-specific configuration."""
        validate_storage_name(self.channel_id, "channel ID")
        if not isinstance(self.source_kind, SensorChannelSource):
            raise TypeError(
                "Channel source kind must be a SensorChannelSource"
            )
        for value, label in (
            (self.topic, "Channel topic"),
            (self.message_type, "Channel message type"),
        ):
            if not isinstance(value, str):
                raise TypeError(f"{label} must be a string")
            if value != value.strip():
                raise ValueError(
                    f"{label} must not contain surrounding whitespace"
                )
        if self.source_kind == SensorChannelSource.SPOT_GEOMETRY:
            if self.topic or self.message_type:
                raise ValueError(
                    "Spot geometry channels must not define a ROS topic "
                    "or message type"
                )
            return
        if not self.topic:
            raise ValueError("Channel topic must not be empty")
        if not self.message_type:
            raise ValueError("Channel message type must not be empty")
        if not self.topic.startswith("/"):
            raise ValueError("Channel topic must be an absolute ROS topic")
        if any(character.isspace() for character in self.topic):
            raise ValueError("Channel topic must not contain whitespace")
        message_type_parts = self.message_type.split("/")
        if (
            len(message_type_parts) != 3
            or message_type_parts[1] != "msg"
            or not all(message_type_parts)
        ):
            raise ValueError(
                "Channel message type must use package/msg/Type format"
            )

    def to_dict(self) -> Dict[str, str]:
        """Serialize the channel for the sensor YAML document."""
        return {
            "channel_id": self.channel_id,
            "topic": self.topic,
            "message_type": self.message_type,
            "source_kind": self.source_kind.value,
        }


@dataclass(frozen=True)
class SensorDefinition:
    """One physical sensor mounted relative to the Spot hand."""

    sensor_id: str
    display_name: str
    hand_to_probe: PoseData
    channels: Tuple[SensorChannel, ...] = ()

    @property
    def probe_frame(self) -> str:
        """Return the derived TF child frame."""
        return sensor_probe_frame(self.sensor_id)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "SensorDefinition":
        """Create a definition from serialized YAML data."""
        if not isinstance(data, dict):
            raise ValueError("sensor definition must be an object")
        raw_channels = data.get("channels", ())
        if not isinstance(raw_channels, (list, tuple)):
            raise ValueError("sensor channels must be an array")
        return cls(
            sensor_id=str(data["sensor_id"]),
            display_name=str(data["display_name"]),
            hand_to_probe=PoseData.from_dict(data["hand_to_probe"]),
            channels=tuple(
                SensorChannel.from_dict(channel)
                for channel in raw_channels
            ),
        )

    def validate(self) -> None:
        """Validate sensor identity and hand-to-probe transform."""
        if self.sensor_id == BARE_HAND_MOTION_ID:
            raise ValueError(
                "Sensor ID 'hand' is reserved for the robot hand frame"
            )
        sensor_probe_frame(self.sensor_id)
        if not self.display_name.strip():
            raise ValueError("Sensor display name must not be empty")
        if self.display_name != self.display_name.strip():
            raise ValueError(
                "Sensor display name must not contain surrounding whitespace"
            )
        self.hand_to_probe.validate()
        seen_channel_ids = set()
        for channel in self.channels:
            if not isinstance(channel, SensorChannel):
                raise TypeError(
                    "Sensor channels must contain SensorChannel values"
                )
            channel.validate()
            if channel.channel_id in seen_channel_ids:
                raise ValueError(
                    f"Duplicate channel ID: {channel.channel_id}"
                )
            seen_channel_ids.add(channel.channel_id)

    def to_dict(self) -> Dict[str, Any]:
        """Serialize the sensor definition."""
        return {
            "sensor_id": self.sensor_id,
            "display_name": self.display_name,
            "hand_to_probe": self.hand_to_probe.to_dict(),
            "channels": [
                channel.to_dict() for channel in self.channels
            ],
        }


@dataclass(frozen=True)
class MotionAttachmentSnapshot:
    """Freeze effective hand-to-probe geometry for one motion workflow."""

    sensor_id: str
    display_name: str
    probe_frame: str
    attachment_revision: int
    hand_to_probe_position: Tuple[float, float, float]
    hand_to_probe_orientation: Tuple[float, float, float, float]

    def __post_init__(self) -> None:
        if (
            isinstance(self.attachment_revision, bool)
            or not isinstance(self.attachment_revision, int)
            or self.attachment_revision < 0
        ):
            raise ValueError(
                "Attachment revision must be a non-negative integer"
            )

    @property
    def has_sensor(self) -> bool:
        """Return whether a physical registered sensor is attached."""
        return bool(self.sensor_id)

    @property
    def motion_sensor_id(self) -> str:
        """Return the transient command identity for effective probe motion."""
        return self.sensor_id or BARE_HAND_MOTION_ID

    @classmethod
    def from_definition(
        cls,
        definition: SensorDefinition,
        attachment_revision: int,
    ) -> "MotionAttachmentSnapshot":
        """Freeze one validated physical sensor definition."""
        definition.validate()
        position = definition.hand_to_probe.position
        orientation = definition.hand_to_probe.orientation
        return cls(
            sensor_id=definition.sensor_id,
            display_name=definition.display_name,
            probe_frame=definition.probe_frame,
            attachment_revision=attachment_revision,
            hand_to_probe_position=(
                float(position.x),
                float(position.y),
                float(position.z),
            ),
            hand_to_probe_orientation=(
                float(orientation.x),
                float(orientation.y),
                float(orientation.z),
                float(orientation.w),
            ),
        )

    @classmethod
    def bare_hand(
        cls,
        attachment_revision: int,
    ) -> "MotionAttachmentSnapshot":
        """Return identity geometry for Spot's bare hand frame."""
        return cls(
            sensor_id="",
            display_name="No sensor",
            probe_frame=SENSOR_PARENT_FRAME,
            attachment_revision=attachment_revision,
            hand_to_probe_position=(0.0, 0.0, 0.0),
            hand_to_probe_orientation=(0.0, 0.0, 0.0, 1.0),
        )

    def hand_to_probe(self) -> PoseData:
        """Return an independent pose for motion calculations."""
        return PoseData(
            position=Vector3Data(*self.hand_to_probe_position),
            orientation=QuaternionData(*self.hand_to_probe_orientation),
        )


def sensor_definition_from_values(
    sensor_id: str,
    display_name: str,
    x: float,
    y: float,
    z: float,
    roll_degrees: float,
    pitch_degrees: float,
    yaw_degrees: float,
    channels: Tuple[SensorChannel, ...] = (),
) -> SensorDefinition:
    """Create and validate a sensor definition from UI transform values."""
    definition = SensorDefinition(
        sensor_id=sensor_id,
        display_name=display_name,
        channels=tuple(channels),
        hand_to_probe=PoseData(
            position=Vector3Data(x=x, y=y, z=z),
            orientation=quaternion_from_rpy_degrees(
                roll_degrees,
                pitch_degrees,
                yaw_degrees,
            ),
        ),
    )
    definition.validate()
    return definition
