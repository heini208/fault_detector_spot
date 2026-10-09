"""Resolve saved routine navigation settings into shared runtime commands."""

from fault_detector_msgs.msg import OperationalIntent

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.commanding.semantic_command import (
    InspectionSelection,
    SemanticCommand,
)


def routine_navigation_command(intent, object_repository, map_repository, map_artifacts):
    """Snapshot routine navigation targets without requiring an idle robot."""
    launch = intent.intent == OperationalIntent.INTENT_LAUNCH_ROUTINE_MAP
    if not launch and intent.intent != OperationalIntent.INTENT_MOVE_TO_ROUTINE_WAYPOINT:
        raise ValueError("Unsupported routine navigation operation")
    if object_repository is None:
        raise RuntimeError("Inspection object data is unavailable")
    definition = object_repository.load(intent.object_id)
    definition.validate()
    routine = definition.get_routine(intent.routine_id)
    if routine is None:
        raise ValueError("The selected routine does not exist for this object")
    if not routine.map_id:
        raise ValueError("The selected routine has no saved map")
    if not launch and not routine.waypoint_id:
        raise ValueError("The selected routine has no saved waypoint")
    if map_repository is None:
        raise RuntimeError("Map navigation data is unavailable")
    map_definition = map_repository.load(routine.map_id)
    map_definition.validate()
    if launch:
        if map_artifacts is None:
            raise RuntimeError("Map database artifacts are unavailable")
        if not any(path.name == f"{routine.map_id}.db"
                   for path in map_artifacts.paths(routine.map_id)):
            raise FileNotFoundError(
                f"Database file not found for map: {routine.map_id}"
            )
    elif map_definition.get_waypoint(routine.waypoint_id) is None:
        raise ValueError(
            f"Waypoint '{routine.waypoint_id}' does not exist in map '{routine.map_id}'"
        )
    return SemanticCommand(
        command_id=CommandID.START_LOCALIZATION if launch else CommandID.MOVE_TO_WAYPOINT,
        map_name=routine.map_id,
        waypoint_name="" if launch else routine.waypoint_id,
        inspection=InspectionSelection(
            object_id=intent.object_id,
            routine_id=intent.routine_id,
        ),
    )


__all__ = ["routine_navigation_command"]
