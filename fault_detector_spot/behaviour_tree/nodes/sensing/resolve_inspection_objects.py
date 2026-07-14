"""Resolve inspection objects from landmarks and live tags."""

from pathlib import Path
from typing import Any, Optional

import py_trees

from fault_detector_spot.inspection.map_repository import (
    MapRepository,
)
from fault_detector_spot.inspection.models import MapDefinition
from fault_detector_spot.inspection.object_pose_resolver import (
    ObjectPoseResolver,
    ObjectPoseState,
)
from fault_detector_spot.inspection.tag_element_adapter import (
    tag_elements_to_pose_stamped,
)


class ResolveInspectionObjects(
    py_trees.behaviour.Behaviour,
):
    """Resolve all inspection object poses for the active map."""

    def __init__(
        self,
        slam_helper,
        name="ResolveInspectionObjects",
    ):
        super().__init__(name)
        self.slam_helper = slam_helper
        self.node = None
        self.repository = None
        self.resolver = ObjectPoseResolver()

        self.blackboard = self.attach_blackboard_client()

        self.cached_map_name: Optional[str] = None
        self.cached_map_mtime_ns: Optional[int] = None
        self.cached_map_definition: Optional[
            MapDefinition
        ] = None

        self.last_error = ""

    def setup(self, **kwargs: Any) -> None:
        """Register blackboard access and repositories."""
        self.node = kwargs["node"]

        self.blackboard.register_key(
            "active_map_name",
            access=py_trees.common.Access.READ,
        )
        self.blackboard.register_key(
            "visible_tags_map_frame",
            access=py_trees.common.Access.READ,
        )
        self.blackboard.register_key(
            "inspection_objects",
            access=py_trees.common.Access.WRITE,
        )
        self.blackboard.register_key(
            "inspection_object_error",
            access=py_trees.common.Access.WRITE,
        )

        self.blackboard.inspection_objects = {}
        self.blackboard.inspection_object_error = ""

        self.repository = MapRepository(
            self.slam_helper.maps_dir
        )

    def update(self):
        """Resolve object states for the active map."""
        map_name = self._get_active_map_name()

        if not map_name:
            self._clear_results()
            self.feedback_message = "No active map"
            return py_trees.common.Status.SUCCESS

        try:
            map_definition = self._load_map_definition(
                map_name
            )

            visible_tags = self._get_visible_tags()
            live_marker_poses = (
                tag_elements_to_pose_stamped(
                    visible_tags
                )
            )

            resolved_objects = self.resolver.resolve_all(
                map_definition,
                live_marker_poses,
            )

        except (
            FileNotFoundError,
            OSError,
            TypeError,
            ValueError,
        ) as exception:
            self._set_error(str(exception))
            return py_trees.common.Status.SUCCESS

        self.blackboard.inspection_objects = (
            resolved_objects
        )
        self.blackboard.inspection_object_error = ""
        self.last_error = ""

        live_count = sum(
            result.state == ObjectPoseState.LIVE
            for result in resolved_objects.values()
        )
        remembered_count = sum(
            result.state == ObjectPoseState.REMEMBERED
            for result in resolved_objects.values()
        )
        unavailable_count = sum(
            result.state == ObjectPoseState.UNAVAILABLE
            for result in resolved_objects.values()
        )
        invalid_count = sum(
            result.state == ObjectPoseState.INVALID
            for result in resolved_objects.values()
        )

        self.feedback_message = (
            f"Objects: {live_count} live, "
            f"{remembered_count} remembered, "
            f"{unavailable_count} unavailable, "
            f"{invalid_count} invalid"
        )

        return py_trees.common.Status.SUCCESS

    def _get_active_map_name(self) -> Optional[str]:
        """Return the active map name."""
        if not self.blackboard.exists(
            "active_map_name"
        ):
            return None

        return self.blackboard.active_map_name

    def _get_visible_tags(self):
        """Return current map-frame tag observations."""
        if not self.blackboard.exists(
            "visible_tags_map_frame"
        ):
            return {}

        return (
            self.blackboard.visible_tags_map_frame
            or {}
        )

    def _load_map_definition(
        self,
        map_name: str,
    ) -> MapDefinition:
        """Load and cache active map metadata."""
        path = self.repository.get_map_path(
            map_name
        )
        modified_ns = Path(path).stat().st_mtime_ns

        cache_is_current = (
            self.cached_map_name == map_name
            and self.cached_map_mtime_ns
            == modified_ns
            and self.cached_map_definition
            is not None
        )

        if not cache_is_current:
            self.cached_map_definition = (
                self.repository.load(map_name)
            )
            self.cached_map_name = map_name
            self.cached_map_mtime_ns = (
                modified_ns
            )

        return self.cached_map_definition

    def _clear_results(self) -> None:
        """Clear current resolved object states."""
        self.blackboard.inspection_objects = {}
        self.blackboard.inspection_object_error = ""
        self.last_error = ""

    def _set_error(self, message: str) -> None:
        """Store a resolution error without failing sensing."""
        self.blackboard.inspection_objects = {}
        self.blackboard.inspection_object_error = (
            message
        )
        self.feedback_message = message

        if message != self.last_error:
            self.node.get_logger().warning(
                f"Inspection object resolution failed: "
                f"{message}"
            )

        self.last_error = message