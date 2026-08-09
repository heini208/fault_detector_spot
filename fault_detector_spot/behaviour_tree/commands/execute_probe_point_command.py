"""Semantic command selecting one saved probe point."""

from builtin_interfaces.msg import Time

from fault_detector_spot.inspection.repository_utils import (
    validate_storage_name,
)

from .command_ids import CommandID
from .simple_command import SimpleCommand


class ExecuteProbePointCommand(SimpleCommand):
    """Identify one persisted probe point without copying its geometry."""

    def __init__(
        self,
        stamp: Time,
        object_id: str,
        routine_id: str,
        probe_point_id: str,
    ):
        validate_storage_name(object_id, "object ID")
        validate_storage_name(routine_id, "routine ID")
        validate_storage_name(probe_point_id, "probe point ID")
        super().__init__(CommandID.EXECUTE_PROBE_POINT, stamp)
        self.object_id = object_id
        self.routine_id = routine_id
        self.probe_point_id = probe_point_id

    def __repr__(self):
        return (
            "<ExecuteProbePointCommand "
            f"object={self.object_id!r} "
            f"routine={self.routine_id!r} "
            f"point={self.probe_point_id!r}>"
        )
