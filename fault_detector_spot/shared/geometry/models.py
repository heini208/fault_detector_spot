"""Serializable geometry primitives shared across robot domains."""

import math
from dataclasses import dataclass
from typing import Any, Dict


def _require_dict(data: Any, field_name: str) -> Dict[str, Any]:
    if not isinstance(data, dict):
        raise ValueError(f"{field_name} must be an object")
    return data


@dataclass
class Vector3Data:
    """Serializable three-dimensional vector."""

    x: float
    y: float
    z: float

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "Vector3Data":
        data = _require_dict(data, "vector")
        return cls(
            x=float(data["x"]),
            y=float(data["y"]),
            z=float(data["z"]),
        )

    @classmethod
    def zero(cls) -> "Vector3Data":
        return cls(x=0.0, y=0.0, z=0.0)

    def validate(self) -> None:
        if not all(
            math.isfinite(value)
            for value in (self.x, self.y, self.z)
        ):
            raise ValueError("Vector contains a non-finite value")

    def to_dict(self) -> Dict[str, float]:
        return {"x": self.x, "y": self.y, "z": self.z}


@dataclass
class QuaternionData:
    """Serializable normalized quaternion."""

    x: float
    y: float
    z: float
    w: float

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "QuaternionData":
        data = _require_dict(data, "quaternion")
        return cls(
            x=float(data["x"]),
            y=float(data["y"]),
            z=float(data["z"]),
            w=float(data["w"]),
        )

    @classmethod
    def identity(cls) -> "QuaternionData":
        return cls(x=0.0, y=0.0, z=0.0, w=1.0)

    def validate(self) -> None:
        values = (self.x, self.y, self.z, self.w)
        if not all(math.isfinite(value) for value in values):
            raise ValueError(
                "Quaternion contains a non-finite value"
            )
        norm = math.sqrt(sum(value ** 2 for value in values))
        if not math.isclose(norm, 1.0, abs_tol=1e-3):
            raise ValueError("Quaternion must be normalized")

    def to_dict(self) -> Dict[str, float]:
        return {
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "w": self.w,
        }


@dataclass
class PoseData:
    """Serializable position and orientation."""

    position: Vector3Data
    orientation: QuaternionData

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "PoseData":
        data = _require_dict(data, "pose")
        return cls(
            position=Vector3Data.from_dict(data["position"]),
            orientation=QuaternionData.from_dict(
                data["orientation"]
            ),
        )

    @classmethod
    def identity(cls) -> "PoseData":
        return cls(
            position=Vector3Data.zero(),
            orientation=QuaternionData.identity(),
        )

    def validate(self) -> None:
        self.position.validate()
        self.orientation.validate()

    def to_dict(self) -> Dict[str, Any]:
        return {
            "position": self.position.to_dict(),
            "orientation": self.orientation.to_dict(),
        }


