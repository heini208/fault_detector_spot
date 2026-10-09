"""Map-independent inspection object models."""

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Set

from fault_detector_spot.navigation.body_height import validate_body_height
from fault_detector_spot.shared.geometry.models import PoseData, QuaternionData, Vector3Data
from fault_detector_spot.shared.persistence.file_storage import validate_storage_name

from fault_detector_spot.inspection.sensing import (
    surface_distance_validation as _surface_distance_validation,
)


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
class ImagePoint:
    """Persisted source pixel selected in a reference image."""

    u: int
    v: int

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "ImagePoint":
        data = _require_dict(data, "reference_pixel")
        return cls(u=int(data["u"]), v=int(data["v"]))

    def validate(self) -> None:
        if self.u < 0 or self.v < 0:
            raise ValueError(
                "Reference pixel coordinates must not be negative"
            )

    def to_dict(self) -> Dict[str, int]:
        return {"u": self.u, "v": self.v}


@dataclass
class ReferenceTag:
    """AprilTag that defines a routine-local inspection frame."""

    tag_id: int
    tag_family: str

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "ReferenceTag":
        data = _require_dict(data, "reference_tag")
        return cls(
            tag_id=int(data["tag_id"]),
            tag_family=str(data["tag_family"]),
        )

    def validate(self) -> None:
        if self.tag_id < 0:
            raise ValueError("Tag ID must not be negative")
        _require_text(self.tag_family, "Tag family")

    def to_dict(self) -> Dict[str, Any]:
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
        identity = (
            self.view_id,
            self.camera_id,
            self.slot_index,
        )
        if any(value is not None for value in identity):
            if any(value is None for value in identity):
                raise ValueError(
                    "Reference view identity must be complete"
                )
            _require_text(self.view_id, "Reference view ID")
            _require_text(
                self.camera_id,
                "Reference camera ID",
            )
            if (
                isinstance(self.slot_index, bool)
                or not isinstance(self.slot_index, int)
                or self.slot_index < 0
                or self.slot_index > 5
            ):
                raise ValueError(
                    "Reference view slot must be between 0 and 5"
                )
        if (
            self.reference_dataset_path is not None
            and self.view_id is None
        ):
            raise ValueError(
                "Stored reference view requires camera identity"
            )

    def to_dict(self) -> Dict[str, Any]:
        result = {
            "controlled_frame_pose_object": (
                self.controlled_frame_pose_object.to_dict()
            ),
            "controlled_frame": self.controlled_frame,
        }
        if self.reference_dataset_path is not None:
            result[
                "reference_dataset_path"
            ] = self.reference_dataset_path
        if self.view_id is not None:
            result["view_id"] = self.view_id
            result["camera_id"] = self.camera_id
            result["slot_index"] = self.slot_index
        return result


@dataclass
class PreApproachPathPoint:
    """Named probe-tip pose relative to the routine reference tag."""

    name: str
    pose_object: PoseData
    position_tolerance_m: float = 0.01
    arm_speed_scale: float = 1.0

    def validate(self) -> None:
        _require_text(self.name, "Pathing point name")
        validate_arm_speed_scale(self.arm_speed_scale)
        self.pose_object.validate()
        if not math.isfinite(self.position_tolerance_m) or self.position_tolerance_m <= 0:
            raise ValueError("Pathing point tolerance must be positive and finite")

    @classmethod
    def from_dict(cls, data):
        data = _require_dict(data, "pre_approach_path_point")
        point = cls(str(data["name"]), PoseData.from_dict(data["pose_object"]),
                    float(data.get("position_tolerance_m", .01)),
                    float(data.get("arm_speed_scale", 1.0)))
        point.validate()
        return point

    def to_dict(self):
        self.validate()
        return {"name": self.name, "pose_object": self.pose_object.to_dict(),
                "position_tolerance_m": self.position_tolerance_m,
                "arm_speed_scale": self.arm_speed_scale}


@dataclass
class ProbePoint:
    """Persisted probe execution target with local +X pointing inward."""

    probe_point_id: str
    display_name: str
    aligned_preapproach_pose_object: PoseData
    target_surface_distance_m: float
    position_tolerance_m: float
    orientation_tolerance_rad: float
    measurement_duration_sec: float
    aligned_preapproach_distance_m: float
    reference_pixel: Optional[ImagePoint] = None
    reference_view_id: Optional[str] = None
    pre_approach_path: List[PreApproachPathPoint] = field(default_factory=list)
    fully_custom: bool = False
    final_probe_pose_object: Optional[PoseData] = None
    final_probe_path: List[PreApproachPathPoint] = field(default_factory=list)
    pre_approach_speed_scale: float = 1.0
    final_probe_speed_scale: float = .2
    final_position_tolerance_m: float = .01

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "ProbePoint":
        data = _require_dict(data, "probe_point")
        pixel = data.get("reference_pixel")
        reference_view_id = data.get("reference_view_id")
        return cls(
            fully_custom=data.get("fully_custom", False),
            final_probe_pose_object=(PoseData.from_dict(data["final_probe_pose_object"])
                                     if data.get("final_probe_pose_object") is not None else None),
            final_probe_path=[PreApproachPathPoint.from_dict(p) for p in data.get("final_probe_path", [])],
            pre_approach_speed_scale=float(data.get("pre_approach_speed_scale", 1.0)),
            final_probe_speed_scale=float(data.get("final_probe_speed_scale", .2)),
            final_position_tolerance_m=float(data.get("final_position_tolerance_m", .01)),
            pre_approach_path=[
                PreApproachPathPoint.from_dict(point)
                for point in _require_list(
                    data.get("pre_approach_path", []), "pre_approach_path"
                )
            ],
            probe_point_id=str(data["probe_point_id"]),
            display_name=str(data["display_name"]),
            aligned_preapproach_pose_object=PoseData.from_dict(
                data["aligned_preapproach_pose_object"]
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
        )

    def validate(self) -> None:
        _require_text(self.probe_point_id, "Probe point ID")
        _require_text(
            self.display_name,
            "Probe point display name",
        )
        if not isinstance(self.fully_custom, bool):
            raise ValueError("Fully custom mode must be a boolean")
        validate_arm_speed_scale(self.pre_approach_speed_scale)
        validate_arm_speed_scale(self.final_probe_speed_scale)
        if not math.isfinite(self.final_position_tolerance_m) or self.final_position_tolerance_m <= 0:
            raise ValueError("Final pose tolerance must be positive and finite")
        if self.fully_custom:
            if self.final_probe_pose_object is None:
                raise ValueError("Custom probe point requires an explicit final pose")
            self.final_probe_pose_object.validate()
        for point in (*self.pre_approach_path, *self.final_probe_path):
            point.validate()
        self.aligned_preapproach_pose_object.validate()
        numeric_values = (
            self.target_surface_distance_m,
            self.position_tolerance_m,
            self.orientation_tolerance_rad,
            self.measurement_duration_sec,
            self.aligned_preapproach_distance_m,
        )
        if not all(
            math.isfinite(value)
            for value in numeric_values
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
        if not self.fully_custom:
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

    def to_dict(self) -> Dict[str, Any]:
        result = {
            "fully_custom": self.fully_custom,
            "final_probe_pose_object": self.final_probe_pose_object.to_dict() if self.final_probe_pose_object else None,
            "final_probe_path": [point.to_dict() for point in self.final_probe_path],
            "pre_approach_speed_scale": self.pre_approach_speed_scale,
            "final_probe_speed_scale": self.final_probe_speed_scale,
            "final_position_tolerance_m": self.final_position_tolerance_m,
            "pre_approach_path": [point.to_dict() for point in self.pre_approach_path],
            "probe_point_id": self.probe_point_id,
            "display_name": self.display_name,
            "aligned_preapproach_pose_object": (
                self.aligned_preapproach_pose_object.to_dict()
            ),
            "target_surface_distance_m": (
                self.target_surface_distance_m
            ),
            "position_tolerance_m": self.position_tolerance_m,
            "orientation_tolerance_rad": (
                self.orientation_tolerance_rad
            ),
            "measurement_duration_sec": (
                self.measurement_duration_sec
            ),
            "aligned_preapproach_distance_m": (
                self.aligned_preapproach_distance_m
            ),
        }
        if self.reference_pixel is not None:
            result[
                "reference_pixel"
            ] = self.reference_pixel.to_dict()
        if self.reference_view_id is not None:
            result[
                "reference_view_id"
            ] = self.reference_view_id
        return result


@dataclass
class InspectionRoutine:
    """Ordered probe procedure with its own reference tag."""

    routine_id: str
    display_name: str
    reference_tag: ReferenceTag
    base_position: Optional[PoseData] = None
    # Shared probe-tip pose in the object frame, resolved through calibration.
    safe_approach_pose_object: Optional[PoseData] = None
    safe_approach_position_tolerance_m: float = 0.1
    reference_views: List[ReferenceView] = field(
        default_factory=list
    )
    probe_points: List[ProbePoint] = field(
        default_factory=list
    )
    base_body_height_m: float = 0.0
    map_id: str = ""
    waypoint_id: str = ""

    @classmethod
    def from_dict(
        cls,
        data: Dict[str, Any],
    ) -> "InspectionRoutine":
        data = _require_dict(
            data,
            "inspection_routine",
        )
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
            reference_tag=ReferenceTag.from_dict(
                data["reference_tag"]
            ),
            base_position=(
                PoseData.from_dict(data["base_position"])
                if data.get("base_position") is not None
                else None
            ),
            base_body_height_m=float(data.get("base_body_height_m", 0.0)),
            map_id=data.get("map_id", ""),
            waypoint_id=data.get("waypoint_id", ""),
            safe_approach_position_tolerance_m=float(data.get("safe_approach_position_tolerance_m", .1)),
            safe_approach_pose_object=(
                PoseData.from_dict(data["safe_approach_pose_object"])
                if data.get("safe_approach_pose_object") is not None
                else None
            ),
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
        _require_text(self.routine_id, "Routine ID")
        _require_text(
            self.display_name,
            "Routine display name",
        )
        self.reference_tag.validate()
        validate_body_height(self.base_body_height_m)
        for value, label in (
            (self.map_id, "Routine map ID"),
            (self.waypoint_id, "Routine waypoint ID"),
        ):
            if not isinstance(value, str):
                raise TypeError(f"{label} must be a string")
            if value != value.strip():
                raise ValueError(f"{label} must not contain surrounding whitespace")
        if self.map_id:
            validate_storage_name(self.map_id, "Routine map ID")
        if self.waypoint_id and not self.map_id:
            raise ValueError("Routine waypoint requires a map")
        if not math.isfinite(self.safe_approach_position_tolerance_m) or self.safe_approach_position_tolerance_m <= 0:
            raise ValueError("Safe approach tolerance must be positive and finite")
        if self.safe_approach_pose_object is not None:
            self.safe_approach_pose_object.validate()
        if self.base_position is not None:
            self.base_position.validate()
            if (
                abs(self.base_position.position.z) > 1e-9
                or abs(self.base_position.orientation.x) > 1e-9
                or abs(self.base_position.orientation.y) > 1e-9
            ):
                raise ValueError(
                    "Routine base position must be a planar tag-relative pose"
                )
        if not 0 <= len(self.reference_views) <= 6:
            raise ValueError(
                "Routine must contain at most six reference views"
            )
        view_ids: Set[str] = set()
        camera_ids: Set[str] = set()
        slot_indices: Set[int] = set()
        for reference_view in self.reference_views:
            reference_view.validate()
            if (
                reference_view.reference_dataset_path
                is None
            ):
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
                    "Duplicate reference view ID: "
                    f"{reference_view.view_id}"
                )
            if reference_view.camera_id in camera_ids:
                raise ValueError(
                    "Duplicate reference camera ID: "
                    f"{reference_view.camera_id}"
                )
            if reference_view.slot_index in slot_indices:
                raise ValueError(
                    "Duplicate reference view slot: "
                    f"{reference_view.slot_index}"
                )
            view_ids.add(reference_view.view_id)
            camera_ids.add(reference_view.camera_id)
            slot_indices.add(reference_view.slot_index)
        if (
            not self.reference_views
            and self.probe_points
        ):
            raise ValueError(
                "Routine with probe points requires a reference view"
            )
        probe_ids: Set[str] = set()
        for probe_point in self.probe_points:
            probe_point.validate()
            if (
                probe_point.reference_view_id is not None
                and probe_point.reference_view_id
                not in view_ids
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

    def require_safe_approach_pose(self) -> PoseData:
        if self.safe_approach_pose_object is None:
            raise ValueError("Set the routine safe pre-approach arm pose first")
        self.safe_approach_pose_object.validate()
        return self.safe_approach_pose_object

    def get_probe_point(
        self,
        probe_point_id: str,
    ) -> Optional[ProbePoint]:
        return next(
            (
                point
                for point in self.probe_points
                if point.probe_point_id == probe_point_id
            ),
            None,
        )

    def to_dict(self) -> Dict[str, Any]:
        result = {
            "safe_approach_position_tolerance_m": self.safe_approach_position_tolerance_m,
            "routine_id": self.routine_id,
            "display_name": self.display_name,
            "reference_tag": self.reference_tag.to_dict(),
            "reference_views": [
                view.to_dict()
                for view in self.reference_views
            ],
            "probe_points": [
                point.to_dict()
                for point in self.probe_points
            ],
        }
        if self.safe_approach_pose_object is not None:
            result["safe_approach_pose_object"] = (
                self.safe_approach_pose_object.to_dict()
            )
        if self.base_position is not None:
            result["base_position"] = self.base_position.to_dict()
        if self.base_body_height_m != 0.0:
            result["base_body_height_m"] = self.base_body_height_m
        if self.map_id:
            result["map_id"] = self.map_id
        if self.waypoint_id:
            result["waypoint_id"] = self.waypoint_id
        return result


@dataclass
class InspectionObject:
    """Map-independent object containing all inspection routines."""

    object_id: str
    display_name: str
    routines: List[InspectionRoutine] = field(
        default_factory=list
    )

    @classmethod
    def from_dict(
        cls,
        data: Dict[str, Any],
    ) -> "InspectionObject":
        data = _require_dict(
            data,
            "inspection_object",
        )
        routines = _require_list(
            data["routines"],
            "routines",
        )
        return cls(
            object_id=str(data["object_id"]),
            display_name=str(data["display_name"]),
            routines=[
                InspectionRoutine.from_dict(routine)
                for routine in routines
            ],
        )

    def validate(self) -> None:
        _require_text(
            self.object_id,
            "Inspection object ID",
        )
        _require_text(
            self.display_name,
            "Inspection object display name",
        )
        routine_ids: Set[str] = set()
        for routine in self.routines:
            routine.validate()
            if routine.routine_id in routine_ids:
                raise ValueError(
                    "Duplicate routine ID: "
                    f"{routine.routine_id}"
                )
            routine_ids.add(routine.routine_id)

    def get_routine(
        self,
        routine_id: str,
    ) -> Optional[InspectionRoutine]:
        return next(
            (
                routine
                for routine in self.routines
                if routine.routine_id == routine_id
            ),
            None,
        )

    def to_dict(self) -> Dict[str, Any]:
        return {
            "object_id": self.object_id,
            "display_name": self.display_name,
            "routines": [
                routine.to_dict()
                for routine in self.routines
            ],
        }


def validate_arm_speed_scale(value):
    """Limit authored motion speed to the configured arm speed."""
    if isinstance(value, bool) or not math.isfinite(value) or not 0 < value <= 1:
        raise ValueError("Arm speed must be greater than 0% and at most 100%")
