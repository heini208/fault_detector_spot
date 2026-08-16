"""Persistent definitions for hand-mounted inspection sensors."""

import math
import re
from dataclasses import dataclass
from typing import Any, Dict, Tuple

from .models import PoseData, QuaternionData, Vector3Data
from fault_detector_spot.shared.persistence.file_storage import (
    validate_storage_name,
)


SENSOR_PARENT_FRAME = "hand"
BARE_HAND_MOTION_ID = SENSOR_PARENT_FRAME
_SENSOR_ID_PATTERN = re.compile(r"^[a-z][a-z0-9_]*$")


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
class SensorDefinition:
    """One physical sensor mounted relative to the Spot hand."""

    sensor_id: str
    display_name: str
    hand_to_probe: PoseData

    @property
    def probe_frame(self) -> str:
        """Return the derived TF child frame."""
        return sensor_probe_frame(self.sensor_id)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "SensorDefinition":
        """Create a definition from serialized YAML data."""
        if not isinstance(data, dict):
            raise ValueError("sensor definition must be an object")
        return cls(
            sensor_id=str(data["sensor_id"]),
            display_name=str(data["display_name"]),
            hand_to_probe=PoseData.from_dict(data["hand_to_probe"]),
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

    def to_dict(self) -> Dict[str, Any]:
        """Serialize the sensor definition."""
        return {
            "sensor_id": self.sensor_id,
            "display_name": self.display_name,
            "hand_to_probe": self.hand_to_probe.to_dict(),
        }


def sensor_definition_from_values(
    sensor_id: str,
    display_name: str,
    x: float,
    y: float,
    z: float,
    roll_degrees: float,
    pitch_degrees: float,
    yaw_degrees: float,
) -> SensorDefinition:
    """Create and validate a sensor definition from UI transform values."""
    definition = SensorDefinition(
        sensor_id=sensor_id,
        display_name=display_name,
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
