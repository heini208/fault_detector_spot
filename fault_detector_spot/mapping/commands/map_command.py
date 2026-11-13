from builtin_interfaces.msg import Time

from fault_detector_spot.application.behaviour_tree.commands.execution_command import (
    ExecutionCommand,
)


class MapCommand(ExecutionCommand):
    """Execution command carrying a selected map name."""

    def __init__(
        self,
        command_id: str,
        stamp: Time,
        map_name: str,
        mapping_rviz: bool = True,
    ):
        super().__init__(command_id, stamp)
        self.map_name = map_name
        self.mapping_rviz = mapping_rviz
