"""Data models for inspection objects, maps, and probe definitions."""

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional


class WaypointReference(str, Enum):
    """Supported waypoint reference systems."""

    MAP = "map"
    OBJECT = "object"


@dataclass
class Vector3Data:
    """Serializable three-dimensional vector."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    @classmethod
    def from_dict(cls, data: Optional[Dict[str, Any]]) -> "Vector3Data":
        """Create a vector from serialized data."""
        data = data or {}
        return cls(
            x=float(data.get("x", 0.0)),
            y=float(data.get("y", 0.0)),
            z=float(data.get("z", 0.0)),
        )

    def to_dict(self) -> Dict[str, float]:
        """Serialize the vector."""
        return {
            "x": self.x,
            "y": self.y,
            "z": self.z,
        }


@dataclass
class QuaternionData:
    """Serializable quaternion using ROS component ordering."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0

    @classmethod
    def from_dict(
        cls,
        data: Optional[Dict[str, Any]],
    ) -> "QuaternionData":
        """Create a quaternion from serialized data."""
        data = data or {}
        return cls(
            x=float(data.get("x", 0.0)),
            y=float(data.get("y", 0.0)),
            z=float(data.get("z", 0.0)),
            w=float(data.get("w", 1.0)),
        )

    def to_dict(self) -> Dict[str, float]:
        """Serialize the quaternion."""
        return {
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "w": self.w,
        }


@dataclass
class PoseData:
    """Serializable position and orientation."""

    position: Vector3Data = field(default_factory=Vector3Data)
    orientation: QuaternionData = field(default_factory=QuaternionData)

    @classmethod
    def from_dict(cls, data: Optional[Dict[str, Any]]) -> "PoseData":
        """Create a pose from serialized data."""
        data = data or {}
        return cls(
            position=Vector3Data.from_dict(data.get("position")),
            orientation=QuaternionData.from_dict(
                data.get("orientation")
            ),
        )

    def to_dict(self) -> Dict[str, Any]:
        """Serialize the pose."""
        return {
            "position": self.position.to_dict(),
            "orientation": self.orientation.to_dict(),
        }


@dataclass
class ImagePoint:
    """Pixel coordinate in a reference image."""

    u: int
    v: int

    @classmethod
    def from_list(cls, data: List[int]) -> "ImagePoint":
        """Create an image point from a two-element list."""
        if len(data) != 2:
            raise ValueError("Image point must contain u and v")
        return cls(u=int(data[0]), v=int(data[1]))

    def to_list(self) -> List[int]:
        """Serialize the image point."""
        return [self.u, self.v]


@dataclass
class WaypointDefinition:
    """Stored navigation waypoint."""

    name: str
    pose: PoseData
    reference_type: WaypointReference = WaypointReference.MAP
    object_id: Optional[str] = None

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "WaypointDefinition":
        """Create a waypoint from serialized data."""
        return cls(
            name=str(data["name"]),
            pose=PoseData.from_dict(data.get("pose")),
            reference_type=WaypointReference(
                data.get("reference_type", WaypointReference.MAP.value)
            ),
            object_id=data.get("object_id"),
        )

    def validate(self) -> None:
        """Validate waypoint references."""
        if not self.name:
            raise ValueError("Waypoint name must not be empty")

        if (
            self.reference_type == WaypointReference.OBJECT
            and not self.object_id
        ):
            raise ValueError(
                "Object-relative waypoint requires object_id"
            )

    def to_dict(self) -> Dict[str, Any]:
        """Serialize the waypoint."""
        result = {
            "name": self.name,
            "reference_type": self.reference_type.value,
            "pose": self.pose.to_dict(),
        }

        if self.object_id is not None:
            result["object_id"] = self.object_id

        return result


@dataclass
class LandmarkDefinition:
    """Stored marker or localization landmark."""

    name: str
    pose: PoseData
    tag_id: Optional[int] = None

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "LandmarkDefinition":
        """Create a landmark from serialized data."""
        tag_id = data.get("tag_id")
        return cls(
            name=str(data["name"]),
            pose=PoseData.from_dict(data.get("pose")),
            tag_id=int(tag_id) if tag_id is not None else None,
        )

    def validate(self) -> None:
        """Validate the landmark."""
        if not self.name:
            raise ValueError("Landmark name must not be empty")

    def to_dict(self) -> Dict[str, Any]:
        """Serialize the landmark."""
        result = {
            "name": self.name,
            "pose": self.pose.to_dict(),
        }

        if self.tag_id is not None:
            result["tag_id"] = self.tag_id

        return result


@dataclass
class InspectionObject:
    """Static inspection object linked to a marker landmark."""

    object_id: str
    display_name: str
    tag_id: int
    landmark_name: str
    marker_to_object: PoseData = field(default_factory=PoseData)
    approach_waypoints: List[str] = field(default_factory=list)
    inspections: List[str] = field(default_factory=list)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "InspectionObject":
        """Create an inspection object from serialized data."""
        object_id = str(data["id"])
        return cls(
            object_id=object_id,
            display_name=str(data.get("display_name", object_id)),
            tag_id=int(data["tag_id"]),
            landmark_name=str(data["landmark_name"]),
            marker_to_object=PoseData.from_dict(
                data.get("marker_to_object")
            ),
            approach_waypoints=list(
                data.get("approach_waypoints", [])
            ),
            inspections=list(data.get("inspections", [])),
        )

    def validate(self) -> None:
        """Validate object identifiers and references."""
        if not self.object_id:
            raise ValueError("Object ID must not be empty")

        if not self.landmark_name:
            raise ValueError("Object requires a landmark name")

        if self.tag_id < 0:
            raise ValueError("Tag ID must not be negative")

    def to_dict(self) -> Dict[str, Any]:
        """Serialize the inspection object."""
        return {
            "id": self.object_id,
            "display_name": self.display_name,
            "tag_id": self.tag_id,
            "landmark_name": self.landmark_name,
            "marker_to_object": self.marker_to_object.to_dict(),
            "approach_waypoints": list(self.approach_waypoints),
            "inspections": list(self.inspections),
        }


@dataclass
class ProbePoint:
    """Probe point stored relative to an inspection object."""

    probe_point_id: str
    surface_point_object: Vector3Data
    surface_normal_object: Vector3Data
    standoff_m: float
    approach_distance_m: float
    position_tolerance_m: float
    orientation_tolerance_rad: float
    measurement_duration_sec: float
    approach_waypoint: Optional[str] = None
    sensor_path: Optional[str] = None
    reference_pixel: Optional[ImagePoint] = None

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "ProbePoint":
        """Create a probe point from serialized data."""
        pixel_data = data.get("reference_pixel")
        return cls(
            probe_point_id=str(data["id"]),
            surface_point_object=Vector3Data.from_dict(
                data.get("surface_point_object")
            ),
            surface_normal_object=Vector3Data.from_dict(
                data.get("surface_normal_object")
            ),
            standoff_m=float(data.get("standoff_m", 0.01)),
            approach_distance_m=float(
                data.get("approach_distance_m", 0.10)
            ),
            position_tolerance_m=float(
                data.get("position_tolerance_m", 0.01)
            ),
            orientation_tolerance_rad=float(
                data.get("orientation_tolerance_rad", 0.087)
            ),
            measurement_duration_sec=float(
                data.get("measurement_duration_sec", 1.0)
            ),
            approach_waypoint=data.get("approach_waypoint"),
            sensor_path=data.get("sensor_path"),
            reference_pixel=(
                ImagePoint.from_list(pixel_data)
                if pixel_data is not None
                else None
            ),
        )

    def validate(self) -> None:
        """Validate probe distances and tolerances."""
        if not self.probe_point_id:
            raise ValueError("Probe point ID must not be empty")

        if self.standoff_m < 0.0:
            raise ValueError("Standoff must not be negative")

        if self.approach_distance_m <= self.standoff_m:
            raise ValueError(
                "Approach distance must exceed standoff"
            )

        if self.position_tolerance_m <= 0.0:
            raise ValueError(
                "Position tolerance must be positive"
            )

        if self.orientation_tolerance_rad <= 0.0:
            raise ValueError(
                "Orientation tolerance must be positive"
            )

    def to_dict(self) -> Dict[str, Any]:
        """Serialize the probe point."""
        result = {
            "id": self.probe_point_id,
            "surface_point_object":
                self.surface_point_object.to_dict(),
            "surface_normal_object":
                self.surface_normal_object.to_dict(),
            "standoff_m": self.standoff_m,
            "approach_distance_m": self.approach_distance_m,
            "position_tolerance_m": self.position_tolerance_m,
            "orientation_tolerance_rad":
                self.orientation_tolerance_rad,
            "measurement_duration_sec":
                self.measurement_duration_sec,
        }

        if self.approach_waypoint is not None:
            result["approach_waypoint"] = self.approach_waypoint

        if self.sensor_path is not None:
            result["sensor_path"] = self.sensor_path

        if self.reference_pixel is not None:
            result["reference_pixel"] = (
                self.reference_pixel.to_list()
            )

        return result


@dataclass
class InspectionDefinition:
    """Complete reusable inspection configuration."""

    inspection_id: str
    map_name: str
    object_id: str
    probe_points: List[ProbePoint] = field(default_factory=list)
    reference_image: Optional[str] = None
    default_approach_waypoint: Optional[str] = None
    schema_version: int = 1

    @classmethod
    def from_dict(
        cls,
        data: Dict[str, Any],
    ) -> "InspectionDefinition":
        """Create an inspection definition from serialized data."""
        return cls(
            inspection_id=str(data["inspection_id"]),
            map_name=str(data["map_name"]),
            object_id=str(data["object_id"]),
            probe_points=[
                ProbePoint.from_dict(point)
                for point in data.get("probe_points", [])
            ],
            reference_image=data.get("reference_image"),
            default_approach_waypoint=data.get(
                "default_approach_waypoint"
            ),
            schema_version=int(data.get("schema_version", 1)),
        )

    def validate(self) -> None:
        """Validate inspection identifiers and probe points."""
        if not self.inspection_id:
            raise ValueError("Inspection ID must not be empty")

        if not self.map_name:
            raise ValueError("Inspection requires a map name")

        if not self.object_id:
            raise ValueError("Inspection requires an object ID")

        probe_ids = set()
        for probe_point in self.probe_points:
            probe_point.validate()

            if probe_point.probe_point_id in probe_ids:
                raise ValueError(
                    "Duplicate probe point ID: "
                    f"{probe_point.probe_point_id}"
                )

            probe_ids.add(probe_point.probe_point_id)

    def to_dict(self) -> Dict[str, Any]:
        """Serialize the inspection definition."""
        result = {
            "schema_version": self.schema_version,
            "inspection_id": self.inspection_id,
            "map_name": self.map_name,
            "object_id": self.object_id,
            "probe_points": [
                point.to_dict()
                for point in self.probe_points
            ],
        }

        if self.reference_image is not None:
            result["reference_image"] = self.reference_image

        if self.default_approach_waypoint is not None:
            result["default_approach_waypoint"] = (
                self.default_approach_waypoint
            )

        return result


@dataclass
class MapDefinition:
    """Extended metadata associated with an RTAB-Map database."""

    waypoints: List[WaypointDefinition] = field(default_factory=list)
    landmarks: List[LandmarkDefinition] = field(default_factory=list)
    objects: List[InspectionObject] = field(default_factory=list)
    schema_version: int = 1
    extra_fields: Dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "MapDefinition":
        """Create a map definition from current or legacy data."""
        known_fields = {
            "schema_version",
            "waypoints",
            "landmarks",
            "objects",
        }

        return cls(
            waypoints=[
                WaypointDefinition.from_dict(waypoint)
                for waypoint in data.get("waypoints", [])
            ],
            landmarks=[
                LandmarkDefinition.from_dict(landmark)
                for landmark in data.get("landmarks", [])
            ],
            objects=[
                InspectionObject.from_dict(object_data)
                for object_data in data.get("objects", [])
            ],
            schema_version=int(data.get("schema_version", 1)),
            extra_fields={
                key: value
                for key, value in data.items()
                if key not in known_fields
            },
        )

    def validate(self) -> None:
        """Validate names and references across the map."""
        waypoint_names = set()
        landmark_names = set()
        object_ids = set()

        for waypoint in self.waypoints:
            waypoint.validate()

            if waypoint.name in waypoint_names:
                raise ValueError(
                    f"Duplicate waypoint name: {waypoint.name}"
                )

            waypoint_names.add(waypoint.name)

        for landmark in self.landmarks:
            landmark.validate()

            if landmark.name in landmark_names:
                raise ValueError(
                    f"Duplicate landmark name: {landmark.name}"
                )

            landmark_names.add(landmark.name)

        for inspection_object in self.objects:
            inspection_object.validate()

            if inspection_object.object_id in object_ids:
                raise ValueError(
                    "Duplicate object ID: "
                    f"{inspection_object.object_id}"
                )

            if (
                inspection_object.landmark_name
                not in landmark_names
            ):
                raise ValueError(
                    "Unknown landmark "
                    f"{inspection_object.landmark_name} "
                    f"for object {inspection_object.object_id}"
                )

            for waypoint_name in (
                inspection_object.approach_waypoints
            ):
                if waypoint_name not in waypoint_names:
                    raise ValueError(
                        "Unknown approach waypoint "
                        f"{waypoint_name} for object "
                        f"{inspection_object.object_id}"
                    )

            object_ids.add(inspection_object.object_id)

        for waypoint in self.waypoints:
            if (
                waypoint.reference_type
                == WaypointReference.OBJECT
                and waypoint.object_id not in object_ids
            ):
                raise ValueError(
                    "Unknown object "
                    f"{waypoint.object_id} for waypoint "
                    f"{waypoint.name}"
                )

    def to_dict(self) -> Dict[str, Any]:
        """Serialize the complete map metadata."""
        result = dict(self.extra_fields)
        result.update({
            "schema_version": self.schema_version,
            "waypoints": [
                waypoint.to_dict()
                for waypoint in self.waypoints
            ],
            "landmarks": [
                landmark.to_dict()
                for landmark in self.landmarks
            ],
            "objects": [
                inspection_object.to_dict()
                for inspection_object in self.objects
            ],
        })
        return result