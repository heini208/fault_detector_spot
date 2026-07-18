"""Resolve remembered and live object poses."""

from copy import deepcopy
from dataclasses import dataclass
from enum import Enum
from typing import Dict, Mapping, Optional

from builtin_interfaces.msg import Time as TimeMessage
from geometry_msgs.msg import PoseStamped

from fault_detector_spot.inspection.models import (
    InspectionObject,
    LandmarkDefinition,
    MapDefinition,
)
from fault_detector_spot.inspection.transform_utils import (
    compose_poses,
    pose_data_to_pose,
)


class ObjectPoseState(str, Enum):
    """Availability state of an inspection object pose."""

    LIVE = "live"
    REMEMBERED = "remembered"
    UNAVAILABLE = "unavailable"
    INVALID = "invalid"


class ObjectPoseSource(str, Enum):
    """Source and intended use of an object pose."""

    LIVE_LOCAL = "live_local"
    LIVE_MAP = "live_map"
    REMEMBERED_MAP = "remembered_map"


@dataclass
class ResolvedObjectPose:
    """Resolved runtime state of one inspection object."""

    object_id: str
    tag_id: Optional[int]
    state: ObjectPoseState
    selected_pose: Optional[PoseStamped] = None
    remembered_pose: Optional[PoseStamped] = None
    live_pose: Optional[PoseStamped] = None
    message: str = ""
    frame_id: str = ""
    source: Optional[ObjectPoseSource] = None
    observation_timestamp: Optional[TimeMessage] = None
    observation_age_sec: Optional[float] = None
    observation_source: str = ""

    @property
    def is_available(self) -> bool:
        """Return whether a usable object pose is available."""
        return self.selected_pose is not None

    @property
    def is_live(self) -> bool:
        """Return whether the selected pose is a live observation."""
        return self.state == ObjectPoseState.LIVE

    @property
    def is_probe_usable(self) -> bool:
        """Return whether the pose is a fresh local probe reference."""
        return (
            self.state == ObjectPoseState.LIVE
            and self.source == ObjectPoseSource.LIVE_LOCAL
            and self.selected_pose is not None
        )


class ObjectPoseResolver:
    """Resolve object poses from map metadata and live markers."""

    def __init__(self, map_frame: str = "map"):
        if not map_frame:
            raise ValueError("Map frame must not be empty")

        self.map_frame = map_frame

    def resolve(
        self,
        map_definition: MapDefinition,
        object_id: str,
        live_marker_poses: Optional[
            Mapping[int, PoseStamped]
        ] = None,
    ) -> ResolvedObjectPose:
        """Resolve one object pose."""
        inspection_object = self._find_object(
            map_definition,
            object_id,
        )

        if inspection_object is None:
            return ResolvedObjectPose(
                object_id=object_id,
                tag_id=None,
                state=ObjectPoseState.INVALID,
                message=f"Unknown object: {object_id}",
                frame_id=self.map_frame,
            )

        landmark = self._find_landmark(
            map_definition,
            inspection_object.landmark_name,
        )

        if (
            landmark is not None
            and landmark.tag_id is not None
            and landmark.tag_id != inspection_object.tag_id
        ):
            return ResolvedObjectPose(
                object_id=inspection_object.object_id,
                tag_id=inspection_object.tag_id,
                state=ObjectPoseState.INVALID,
                message=(
                    f"Object tag {inspection_object.tag_id} "
                    f"does not match landmark tag "
                    f"{landmark.tag_id}"
                ),
                frame_id=self.map_frame,
            )

        try:
            remembered_pose = self._resolve_remembered_pose(
                inspection_object,
                landmark,
            )

            live_pose = self._resolve_live_pose(
                inspection_object,
                live_marker_poses,
            )
        except (TypeError, ValueError) as exception:
            return ResolvedObjectPose(
                object_id=inspection_object.object_id,
                tag_id=inspection_object.tag_id,
                state=ObjectPoseState.INVALID,
                message=str(exception),
                frame_id=self.map_frame,
            )

        if live_pose is not None:
            return ResolvedObjectPose(
                object_id=inspection_object.object_id,
                tag_id=inspection_object.tag_id,
                state=ObjectPoseState.LIVE,
                selected_pose=live_pose,
                remembered_pose=remembered_pose,
                live_pose=live_pose,
                message="Using live marker observation",
                frame_id=self.map_frame,
                source=ObjectPoseSource.LIVE_MAP,
                observation_timestamp=deepcopy(
                    live_pose.header.stamp
                ),
                observation_source="base",
            )

        if remembered_pose is not None:
            return ResolvedObjectPose(
                object_id=inspection_object.object_id,
                tag_id=inspection_object.tag_id,
                state=ObjectPoseState.REMEMBERED,
                selected_pose=remembered_pose,
                remembered_pose=remembered_pose,
                live_pose=None,
                message="Using remembered landmark pose",
                frame_id=self.map_frame,
                source=ObjectPoseSource.REMEMBERED_MAP,
                observation_source="stored_map",
            )

        return ResolvedObjectPose(
            object_id=inspection_object.object_id,
            tag_id=inspection_object.tag_id,
            state=ObjectPoseState.UNAVAILABLE,
            message="No live or remembered object pose available",
            frame_id=self.map_frame,
        )

    def resolve_all(
        self,
        map_definition: MapDefinition,
        live_marker_poses: Optional[
            Mapping[int, PoseStamped]
        ] = None,
    ) -> Dict[str, ResolvedObjectPose]:
        """Resolve every inspection object in a map."""
        return {
            inspection_object.object_id: self.resolve(
                map_definition,
                inspection_object.object_id,
                live_marker_poses,
            )
            for inspection_object in map_definition.objects
        }

    def resolve_global(
        self,
        map_definition: MapDefinition,
        object_id: str,
        live_marker_poses: Optional[
            Mapping[int, PoseStamped]
        ] = None,
    ) -> ResolvedObjectPose:
        """Resolve one global map object."""
        return self.resolve(
            map_definition,
            object_id,
            live_marker_poses,
        )

    def resolve_all_global(
        self,
        map_definition: MapDefinition,
        live_marker_poses: Optional[
            Mapping[int, PoseStamped]
        ] = None,
    ) -> Dict[str, ResolvedObjectPose]:
        """Resolve every global map object."""
        return self.resolve_all(
            map_definition,
            live_marker_poses,
        )

    def _resolve_remembered_pose(
        self,
        inspection_object: InspectionObject,
        landmark: Optional[LandmarkDefinition],
    ) -> Optional[PoseStamped]:
        """Resolve an object from its stored landmark."""
        if landmark is None:
            return None

        marker_pose = PoseStamped()
        marker_pose.header.frame_id = self.map_frame
        marker_pose.pose = pose_data_to_pose(landmark.pose)

        return self._apply_marker_to_object(
            marker_pose,
            inspection_object,
        )

    def _resolve_live_pose(
        self,
        inspection_object: InspectionObject,
        live_marker_poses: Optional[
            Mapping[int, PoseStamped]
        ],
    ) -> Optional[PoseStamped]:
        """Resolve an object from its current marker pose."""
        if not live_marker_poses:
            return None

        marker_pose = live_marker_poses.get(
            inspection_object.tag_id
        )

        if marker_pose is None:
            return None

        if not isinstance(marker_pose, PoseStamped):
            raise TypeError(
                "Live marker pose must be PoseStamped"
            )

        if marker_pose.header.frame_id != self.map_frame:
            raise ValueError(
                f"Live marker {inspection_object.tag_id} "
                f"is in frame '{marker_pose.header.frame_id}', "
                f"expected '{self.map_frame}'"
            )

        return self._apply_marker_to_object(
            marker_pose,
            inspection_object,
        )

    def _apply_marker_to_object(
        self,
        marker_pose: PoseStamped,
        inspection_object: InspectionObject,
    ) -> PoseStamped:
        """Apply the fixed marker-to-object transform."""
        result = PoseStamped()
        result.header = marker_pose.header
        result.pose = compose_poses(
            marker_pose.pose,
            pose_data_to_pose(
                inspection_object.marker_to_object
            ),
        )
        return result

    @staticmethod
    def _find_object(
        map_definition: MapDefinition,
        object_id: str,
    ) -> Optional[InspectionObject]:
        """Find an object by ID."""
        return next(
            (
                inspection_object
                for inspection_object
                in map_definition.objects
                if inspection_object.object_id == object_id
            ),
            None,
        )

    @staticmethod
    def _find_landmark(
        map_definition: MapDefinition,
        landmark_name: str,
    ) -> Optional[LandmarkDefinition]:
        """Find a landmark by name."""
        return next(
            (
                landmark
                for landmark in map_definition.landmarks
                if landmark.name == landmark_name
            ),
            None,
        )
