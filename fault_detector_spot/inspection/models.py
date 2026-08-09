"""Inspection objects and map-specific navigation data."""

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Set

from . import surface_distance_validation as _surface_distance_validation


MINIMUM_ALIGNED_PREAPPROACH_SEPARATION_M = (
    _surface_distance_validation.MINIMUM_ALIGNED_PREAPPROACH_SEPARATION_M
)
validate_surface_distance_pair = (
    _surface_distance_validation.validate_surface_distance_pair
)



def _require_dict(data: Any, field_name: str) -> Dict[str, Any]:
    if not isinstance(data, dict):
        raise ValueError(f"{field_name} must be an object")
    return data


def _require_list(data: Any, field_name: str) -> List[Any]:
    if not isinstance(data, list):
        raise ValueError(f"{field_name} must be a list")
    return data


def _require_text(value: str, field_name: str) -> None:
    if not value.strip():
        raise ValueError(f"{field_name} must not be empty")


@dataclass
class Vector3Data:
    """Serializable three-dimensional vector."""

    x: float
    y: float
    z: float

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "Vector3Data":
        """Create a vector from serialized data."""
        data = _require_dict(data, "vector")
        return cls(
            x=float(data["x"]),
            y=float(data["y"]),
            z=float(data["z"]),
        )

    @classmethod
    def zero(cls) -> "Vector3Data":
        """Create a zero vector."""
        return cls(x=0.0, y=0.0, z=0.0)

    def validate(self) -> None:
        """Validate vector components."""
        if not all(
            math.isfinite(value)
            for value in (self.x, self.y, self.z)
        ):
            raise ValueError("Vector contains a non-finite value")

    def to_dict(self) -> Dict[str, float]:
        """Serialize the vector."""
        return {
            "x": self.x,
            "y": self.y,
            "z": self.z,
        }


@dataclass
class QuaternionData:
    """Serializable normalized quaternion."""

    x: float
    y: float
    z: float
    w: float

    @classmethod
    def from_dict(
        cls,
        data: Dict[str, Any],
    ) -> "QuaternionData":
        """Create a quaternion from serialized data."""
        data = _require_dict(data, "quaternion")
        return cls(
            x=float(data["x"]),
            y=float(data["y"]),
            z=float(data["z"]),
            w=float(data["w"]),
        )

    @classmethod
    def identity(cls) -> "QuaternionData":
        """Create an identity quaternion."""
        return cls(x=0.0, y=0.0, z=0.0, w=1.0)

    def validate(self) -> None:
        """Validate quaternion components and normalization."""
        values = (self.x, self.y, self.z, self.w)
        if not all(math.isfinite(value) for value in values):
            raise ValueError(
                "Quaternion contains a non-finite value"
            )

        norm = math.sqrt(sum(value ** 2 for value in values))
        if not math.isclose(norm, 1.0, abs_tol=1e-3):
            raise ValueError("Quaternion must be normalized")

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

    position: Vector3Data
    orientation: QuaternionData

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "PoseData":
        """Create a pose from serialized data."""
        data = _require_dict(data, "pose")
        return cls(
            position=Vector3Data.from_dict(data["position"]),
            orientation=QuaternionData.from_dict(
                data["orientation"]
            ),
        )

    @classmethod
    def identity(cls) -> "PoseData":
        """Create an identity pose."""
        return cls(
            position=Vector3Data.zero(),
            orientation=QuaternionData.identity(),
        )

    def validate(self) -> None:
        """Validate position and orientation."""
        self.position.validate()
        self.orientation.validate()

    def to_dict(self) -> Dict[str, Any]:
        """Serialize the pose."""
        return {
            "position": self.position.to_dict(),
            "orientation": self.orientation.to_dict(),
        }


@dataclass
class ImagePoint:
    """Pixel coordinate used only for reference-image display."""

    u: int
    v: int

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "ImagePoint":
        """Create an image point from serialized data."""
        data = _require_dict(data, "reference_pixel")
        return cls(u=int(data["u"]), v=int(data["v"]))

    def validate(self) -> None:
        """Validate non-negative pixel coordinates."""
        if self.u < 0 or self.v < 0:
            raise ValueError(
                "Reference pixel coordinates must not be negative"
            )

    def to_dict(self) -> Dict[str, int]:
        """Serialize the image point."""
        return {"u": self.u, "v": self.v}


@dataclass
class ReferenceTag:
    """AprilTag that rigidly defines an inspection object frame."""

    tag_id: int
    tag_family: str

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "ReferenceTag":
        """Create a reference tag from serialized data."""
        data = _require_dict(data, "reference_tag")
        return cls(
            tag_id=int(data["tag_id"]),
            tag_family=str(data["tag_family"]),
        )

    def validate(self) -> None:
        """Validate tag identity."""
        if self.tag_id < 0:
            raise ValueError("Tag ID must not be negative")
        _require_text(self.tag_family, "Tag family")

    def to_dict(self) -> Dict[str, Any]:
        """Serialize the reference tag."""
        return {
            "tag_id": self.tag_id,
            "tag_family": self.tag_family,
        }


@dataclass
class ReferenceView:
    """Saved camera view and its pose in the inspection-object frame."""

    controlled_frame_pose_object: PoseData
    controlled_frame: str
    reference_dataset_path: Optional[str] = None
    view_id: Optional[str] = None
    camera_id: Optional[str] = None
    slot_index: Optional[int] = None

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "ReferenceView":
        """Create a reference view from serialized data."""
        data = _require_dict(data, "reference_view")
        dataset_path = data.get("reference_dataset_path")
        return cls(
            controlled_frame_pose_object=PoseData.from_dict(
                data["controlled_frame_pose_object"]
            ),
            controlled_frame=str(data["controlled_frame"]),
            reference_dataset_path=(
                str(dataset_path)
                if dataset_path is not None
                else None
            ),
            view_id=str(data["view_id"]),
            camera_id=str(data["camera_id"]),
            slot_index=int(data["slot_index"]),
        )

    def validate(self) -> None:
        """Validate overview pose and frame."""
        self.controlled_frame_pose_object.validate()
        _require_text(
            self.controlled_frame,
            "Reference view controlled frame",
        )
        if (
            self.reference_dataset_path is not None
            and not self.reference_dataset_path.strip()
        ):
            raise ValueError(
                "Reference dataset path must not be empty"
            )
        identity = (self.view_id, self.camera_id, self.slot_index)
        if any(value is not None for value in identity):
            if any(value is None for value in identity):
                raise ValueError(
                    "Reference view identity must be complete"
                )
            _require_text(self.view_id, "Reference view ID")
            _require_text(self.camera_id, "Reference camera ID")
            if (
                isinstance(self.slot_index, bool)
                or not isinstance(self.slot_index, int)
                or self.slot_index < 0
                or self.slot_index > 2
            ):
                raise ValueError(
                    "Reference view slot must be between 0 and 2"
                )
        if (
            self.reference_dataset_path is not None
            and self.view_id is None
        ):
            raise ValueError(
                "Stored reference view requires camera identity"
            )

    def to_dict(self) -> Dict[str, Any]:
        """Serialize the reference view."""
        result = {
            "controlled_frame_pose_object":
                self.controlled_frame_pose_object.to_dict(),
            "controlled_frame": self.controlled_frame,
        }
        if self.reference_dataset_path is not None:
            result["reference_dataset_path"] = (
                self.reference_dataset_path
            )
        if self.view_id is not None:
            result["view_id"] = self.view_id
            result["camera_id"] = self.camera_id
            result["slot_index"] = self.slot_index
        return result


@dataclass
class ProbePoint:
    """One ordered target with local positive X facing outward."""

    probe_point_id: str
    display_name: str
    safe_approach_pose_object: PoseData
    probe_pose_object: PoseData
    target_surface_distance_m: float
    position_tolerance_m: float
    orientation_tolerance_rad: float
    measurement_duration_sec: float
    aligned_preapproach_distance_m: float
    reference_pixel: Optional[ImagePoint] = None
    reference_view_id: Optional[str] = None
    sensor_path: Optional[str] = None

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "ProbePoint":
        """Create a probe point from serialized data."""
        data = _require_dict(data, "probe_point")
        pixel = data.get("reference_pixel")
        reference_view_id = data.get("reference_view_id")
        sensor_path = data.get("sensor_path")
        return cls(
            probe_point_id=str(data["probe_point_id"]),
            display_name=str(data["display_name"]),
            safe_approach_pose_object=PoseData.from_dict(
                data["safe_approach_pose_object"]
            ),
            probe_pose_object=PoseData.from_dict(
                data["probe_pose_object"]
            ),
            target_surface_distance_m=float(
                data["target_surface_distance_m"]
            ),
            position_tolerance_m=float(
                data["position_tolerance_m"]
            ),
            orientation_tolerance_rad=float(
                data["orientation_tolerance_rad"]
            ),
            measurement_duration_sec=float(
                data["measurement_duration_sec"]
            ),
            aligned_preapproach_distance_m=float(
                data["aligned_preapproach_distance_m"]
            ),
            reference_pixel=(
                ImagePoint.from_dict(pixel)
                if pixel is not None
                else None
            ),
            reference_view_id=(
                str(reference_view_id)
                if reference_view_id is not None
                else None
            ),
            sensor_path=(
                str(sensor_path)
                if sensor_path is not None
                else None
            ),
        )

    def validate(self) -> None:
        """Validate probe pose, distance, and tolerances."""
        _require_text(self.probe_point_id, "Probe point ID")
        _require_text(self.display_name, "Probe point display name")
        self.safe_approach_pose_object.validate()
        self.probe_pose_object.validate()

        numeric_values = (
            self.target_surface_distance_m,
            self.position_tolerance_m,
            self.orientation_tolerance_rad,
            self.measurement_duration_sec,
            self.aligned_preapproach_distance_m,
        )
        if not all(
            math.isfinite(value) for value in numeric_values
        ):
            raise ValueError(
                "Probe point contains a non-finite numeric value"
            )
        if self.position_tolerance_m <= 0.0:
            raise ValueError(
                "Position tolerance must be positive"
            )
        if self.orientation_tolerance_rad <= 0.0:
            raise ValueError(
                "Orientation tolerance must be positive"
            )
        if self.measurement_duration_sec <= 0.0:
            raise ValueError(
                "Measurement duration must be positive"
            )
        validate_surface_distance_pair(
            self.target_surface_distance_m,
            self.aligned_preapproach_distance_m,
        )
        if self.reference_pixel is not None:
            self.reference_pixel.validate()
            if self.reference_view_id is None:
                raise ValueError(
                    "Reference pixel requires a reference view ID"
                )
        if self.reference_view_id is not None:
            _require_text(
                self.reference_view_id,
                "Probe point reference view ID",
            )
        if self.sensor_path is not None:
            _require_text(self.sensor_path, "Sensor path")

    def to_dict(self) -> Dict[str, Any]:
        """Serialize the probe point."""
        result = {
            "probe_point_id": self.probe_point_id,
            "display_name": self.display_name,
            "safe_approach_pose_object":
                self.safe_approach_pose_object.to_dict(),
            "probe_pose_object": self.probe_pose_object.to_dict(),
            "target_surface_distance_m":
                self.target_surface_distance_m,
            "position_tolerance_m": self.position_tolerance_m,
            "orientation_tolerance_rad":
                self.orientation_tolerance_rad,
            "measurement_duration_sec":
                self.measurement_duration_sec,
            "aligned_preapproach_distance_m":
                self.aligned_preapproach_distance_m,
        }
        if self.reference_pixel is not None:
            result["reference_pixel"] = (
                self.reference_pixel.to_dict()
            )
        if self.reference_view_id is not None:
            result["reference_view_id"] = self.reference_view_id
        if self.sensor_path is not None:
            result["sensor_path"] = self.sensor_path
        return result


@dataclass
class InspectionRoutine:
    """Ordered probe procedure for one sensor and up to three views."""

    routine_id: str
    display_name: str
    sensor_id: str
    reference_views: List[ReferenceView] = field(default_factory=list)
    probe_points: List[ProbePoint] = field(default_factory=list)

    @property
    def reference_view(self) -> Optional[ReferenceView]:
        """Return the first display view, if one exists."""
        return self.reference_views[0] if self.reference_views else None

    @reference_view.setter
    def reference_view(self, value: Optional[ReferenceView]) -> None:
        """Replace the complete view list with one display view."""
        self.reference_views = [] if value is None else [value]

    @classmethod
    def from_dict(
        cls,
        data: Dict[str, Any],
    ) -> "InspectionRoutine":
        """Create an inspection routine from serialized data."""
        data = _require_dict(data, "inspection_routine")
        probe_points = _require_list(
            data["probe_points"],
            "probe_points",
        )
        reference_views = _require_list(
            data["reference_views"],
            "reference_views",
        )
        return cls(
            routine_id=str(data["routine_id"]),
            display_name=str(data["display_name"]),
            sensor_id=str(data["sensor_id"]),
            reference_views=[
                ReferenceView.from_dict(view)
                for view in reference_views
            ],
            probe_points=[
                ProbePoint.from_dict(point)
                for point in probe_points
            ],
        )

    def validate(self) -> None:
        """Validate routine identity, frames, and probe points."""
        _require_text(self.routine_id, "Routine ID")
        _require_text(self.display_name, "Routine display name")
        _require_text(self.sensor_id, "Sensor ID")
        if not 0 <= len(self.reference_views) <= 3:
            raise ValueError(
                "Routine must contain at most three reference views"
            )
        view_ids: Set[str] = set()
        camera_ids: Set[str] = set()
        slot_indices: Set[int] = set()
        for reference_view in self.reference_views:
            reference_view.validate()
            if reference_view.reference_dataset_path is None:
                raise ValueError(
                    "Routine reference view requires a dataset path"
                )
            if reference_view.view_id is None:
                raise ValueError(
                    "Routine reference view requires an ID"
                )
            if reference_view.camera_id is None:
                raise ValueError(
                    "Routine reference view requires a camera ID"
                )
            if reference_view.slot_index is None:
                raise ValueError(
                    "Routine reference view requires a slot"
                )
            if reference_view.view_id in view_ids:
                raise ValueError(
                    f"Duplicate reference view ID: "
                    f"{reference_view.view_id}"
                )
            if reference_view.camera_id in camera_ids:
                raise ValueError(
                    f"Duplicate reference camera ID: "
                    f"{reference_view.camera_id}"
                )
            if reference_view.slot_index in slot_indices:
                raise ValueError(
                    f"Duplicate reference view slot: "
                    f"{reference_view.slot_index}"
                )
            view_ids.add(reference_view.view_id)
            camera_ids.add(reference_view.camera_id)
            slot_indices.add(reference_view.slot_index)
        if not self.reference_views and self.probe_points:
            raise ValueError(
                "Routine with probe points requires a reference view"
            )

        probe_ids: Set[str] = set()
        for probe_point in self.probe_points:
            probe_point.validate()
            if (
                probe_point.reference_view_id is not None
                and probe_point.reference_view_id not in view_ids
            ):
                raise ValueError(
                    "Probe point references an unknown reference view: "
                    f"{probe_point.reference_view_id}"
                )
            if probe_point.probe_point_id in probe_ids:
                raise ValueError(
                    "Duplicate probe point ID: "
                    f"{probe_point.probe_point_id}"
                )
            probe_ids.add(probe_point.probe_point_id)

    def get_probe_point(
        self,
        probe_point_id: str,
    ) -> Optional[ProbePoint]:
        """Find a probe point by ID."""
        return next(
            (
                point
                for point in self.probe_points
                if point.probe_point_id == probe_point_id
            ),
            None,
        )

    def to_dict(self) -> Dict[str, Any]:
        """Serialize the inspection routine."""
        return {
            "routine_id": self.routine_id,
            "display_name": self.display_name,
            "sensor_id": self.sensor_id,
            "reference_views": [
                view.to_dict() for view in self.reference_views
            ],
            "probe_points": [
                point.to_dict() for point in self.probe_points
            ],
        }


@dataclass
class InspectionObject:
    """Map-independent object containing all inspection routines."""

    object_id: str
    display_name: str
    reference_tag: ReferenceTag
    routines: List[InspectionRoutine] = field(default_factory=list)

    @classmethod
    def from_dict(
        cls,
        data: Dict[str, Any],
    ) -> "InspectionObject":
        """Create an inspection object from serialized data."""
        data = _require_dict(data, "inspection_object")
        routines = _require_list(data["routines"], "routines")
        return cls(
            object_id=str(data["object_id"]),
            display_name=str(data["display_name"]),
            reference_tag=ReferenceTag.from_dict(
                data["reference_tag"]
            ),
            routines=[
                InspectionRoutine.from_dict(routine)
                for routine in routines
            ],
        )

    def validate(self) -> None:
        """Validate object identity, tag, and routines."""
        _require_text(self.object_id, "Inspection object ID")
        _require_text(
            self.display_name,
            "Inspection object display name",
        )
        self.reference_tag.validate()

        routine_ids: Set[str] = set()
        for routine in self.routines:
            routine.validate()
            if routine.routine_id in routine_ids:
                raise ValueError(
                    f"Duplicate routine ID: {routine.routine_id}"
                )
            routine_ids.add(routine.routine_id)

    def get_routine(
        self,
        routine_id: str,
    ) -> Optional[InspectionRoutine]:
        """Find an inspection routine by ID."""
        return next(
            (
                routine
                for routine in self.routines
                if routine.routine_id == routine_id
            ),
            None,
        )

    def to_dict(self) -> Dict[str, Any]:
        """Serialize the complete inspection object."""
        return {
            "object_id": self.object_id,
            "display_name": self.display_name,
            "reference_tag": self.reference_tag.to_dict(),
            "routines": [
                routine.to_dict() for routine in self.routines
            ],
        }


@dataclass
class Waypoint:
    """Map-relative navigation waypoint."""

    waypoint_id: str
    display_name: str
    pose_map: PoseData

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "Waypoint":
        """Create a waypoint from serialized data."""
        data = _require_dict(data, "waypoint")
        return cls(
            waypoint_id=str(data["waypoint_id"]),
            display_name=str(data["display_name"]),
            pose_map=PoseData.from_dict(data["pose_map"]),
        )

    def validate(self) -> None:
        """Validate waypoint identity and pose."""
        _require_text(self.waypoint_id, "Waypoint ID")
        _require_text(self.display_name, "Waypoint display name")
        self.pose_map.validate()

    def to_dict(self) -> Dict[str, Any]:
        """Serialize the waypoint."""
        return {
            "waypoint_id": self.waypoint_id,
            "display_name": self.display_name,
            "pose_map": self.pose_map.to_dict(),
        }


@dataclass
class LocalizationLandmark:
    """Map-relative AprilTag used only for localization."""

    landmark_id: str
    display_name: str
    reference_tag: ReferenceTag
    pose_map: PoseData

    @classmethod
    def from_dict(
        cls,
        data: Dict[str, Any],
    ) -> "LocalizationLandmark":
        """Create a localization landmark from serialized data."""
        data = _require_dict(data, "localization_landmark")
        return cls(
            landmark_id=str(data["landmark_id"]),
            display_name=str(data["display_name"]),
            reference_tag=ReferenceTag.from_dict(
                data["reference_tag"]
            ),
            pose_map=PoseData.from_dict(data["pose_map"]),
        )

    def validate(self) -> None:
        """Validate landmark identity, tag, and pose."""
        _require_text(self.landmark_id, "Landmark ID")
        _require_text(self.display_name, "Landmark display name")
        self.reference_tag.validate()
        self.pose_map.validate()

    def to_dict(self) -> Dict[str, Any]:
        """Serialize the localization landmark."""
        return {
            "landmark_id": self.landmark_id,
            "display_name": self.display_name,
            "reference_tag": self.reference_tag.to_dict(),
            "pose_map": self.pose_map.to_dict(),
        }


@dataclass
class ObjectApproach:
    """Map waypoint associated with an inspection object."""

    approach_id: str
    display_name: str
    object_id: str
    waypoint_id: str

    @classmethod
    def from_dict(
        cls,
        data: Dict[str, Any],
    ) -> "ObjectApproach":
        """Create an object approach from serialized data."""
        data = _require_dict(data, "object_approach")
        return cls(
            approach_id=str(data["approach_id"]),
            display_name=str(data["display_name"]),
            object_id=str(data["object_id"]),
            waypoint_id=str(data["waypoint_id"]),
        )

    def validate(self) -> None:
        """Validate approach identity and references."""
        _require_text(self.approach_id, "Object approach ID")
        _require_text(
            self.display_name,
            "Object approach display name",
        )
        _require_text(self.object_id, "Object approach object ID")
        _require_text(
            self.waypoint_id,
            "Object approach waypoint ID",
        )

    def to_dict(self) -> Dict[str, Any]:
        """Serialize the object approach."""
        return {
            "approach_id": self.approach_id,
            "display_name": self.display_name,
            "object_id": self.object_id,
            "waypoint_id": self.waypoint_id,
        }


@dataclass
class MapDefinition:
    """Map-specific navigation and localization metadata."""

    map_id: str
    display_name: str
    waypoints: List[Waypoint] = field(default_factory=list)
    localization_landmarks: List[
        LocalizationLandmark
    ] = field(default_factory=list)
    object_approaches: List[ObjectApproach] = field(
        default_factory=list
    )

    @classmethod
    def from_dict(
        cls,
        data: Dict[str, Any],
    ) -> "MapDefinition":
        """Create map metadata from serialized data."""
        data = _require_dict(data, "map_definition")
        waypoints = _require_list(data["waypoints"], "waypoints")
        landmarks = _require_list(
            data["localization_landmarks"],
            "localization_landmarks",
        )
        approaches = _require_list(
            data["object_approaches"],
            "object_approaches",
        )
        return cls(
            map_id=str(data["map_id"]),
            display_name=str(data["display_name"]),
            waypoints=[
                Waypoint.from_dict(waypoint)
                for waypoint in waypoints
            ],
            localization_landmarks=[
                LocalizationLandmark.from_dict(landmark)
                for landmark in landmarks
            ],
            object_approaches=[
                ObjectApproach.from_dict(approach)
                for approach in approaches
            ],
        )

    def validate(self) -> None:
        """Validate map identities and internal references."""
        _require_text(self.map_id, "Map ID")
        _require_text(self.display_name, "Map display name")

        waypoint_ids: Set[str] = set()
        for waypoint in self.waypoints:
            waypoint.validate()
            if waypoint.waypoint_id in waypoint_ids:
                raise ValueError(
                    f"Duplicate waypoint ID: {waypoint.waypoint_id}"
                )
            waypoint_ids.add(waypoint.waypoint_id)

        landmark_ids: Set[str] = set()
        landmark_tags = set()
        for landmark in self.localization_landmarks:
            landmark.validate()
            if landmark.landmark_id in landmark_ids:
                raise ValueError(
                    "Duplicate landmark ID: "
                    f"{landmark.landmark_id}"
                )
            tag_key = (
                landmark.reference_tag.tag_family,
                landmark.reference_tag.tag_id,
            )
            if tag_key in landmark_tags:
                raise ValueError(
                    "Duplicate localization landmark tag: "
                    f"{tag_key[0]}:{tag_key[1]}"
                )
            landmark_ids.add(landmark.landmark_id)
            landmark_tags.add(tag_key)

        approach_ids: Set[str] = set()
        for approach in self.object_approaches:
            approach.validate()
            if approach.approach_id in approach_ids:
                raise ValueError(
                    "Duplicate object approach ID: "
                    f"{approach.approach_id}"
                )
            if approach.waypoint_id not in waypoint_ids:
                raise ValueError(
                    f"Unknown waypoint {approach.waypoint_id} "
                    f"for approach {approach.approach_id}"
                )
            approach_ids.add(approach.approach_id)

    def validate_object_references(
        self,
        known_object_ids: Set[str],
    ) -> None:
        """Validate object IDs against an object repository."""
        for approach in self.object_approaches:
            if approach.object_id not in known_object_ids:
                raise ValueError(
                    f"Unknown inspection object {approach.object_id} "
                    f"for approach {approach.approach_id}"
                )

    def get_waypoint(
        self,
        waypoint_id: str,
    ) -> Optional[Waypoint]:
        """Find a waypoint by ID."""
        return next(
            (
                waypoint
                for waypoint in self.waypoints
                if waypoint.waypoint_id == waypoint_id
            ),
            None,
        )

    def get_landmark(
        self,
        landmark_id: str,
    ) -> Optional[LocalizationLandmark]:
        """Find a localization landmark by ID."""
        return next(
            (
                landmark
                for landmark in self.localization_landmarks
                if landmark.landmark_id == landmark_id
            ),
            None,
        )

    def get_object_approach(
        self,
        approach_id: str,
    ) -> Optional[ObjectApproach]:
        """Find an object approach by ID."""
        return next(
            (
                approach
                for approach in self.object_approaches
                if approach.approach_id == approach_id
            ),
            None,
        )

    def to_dict(self) -> Dict[str, Any]:
        """Serialize the complete map definition."""
        return {
            "map_id": self.map_id,
            "display_name": self.display_name,
            "waypoints": [
                waypoint.to_dict() for waypoint in self.waypoints
            ],
            "localization_landmarks": [
                landmark.to_dict()
                for landmark in self.localization_landmarks
            ],
            "object_approaches": [
                approach.to_dict()
                for approach in self.object_approaches
            ],
        }
