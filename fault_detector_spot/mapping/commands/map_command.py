from builtin_interfaces.msg import Time

from fault_detector_spot.application.commanding.simple_command import (
    SimpleCommand,
)


class MapCommand(SimpleCommand):
    """Execution command carrying a selected map name."""

    def __init__(
        self,
        command_id: str,
        stamp: Time,
        map_name: str,
    ):
        super().__init__(command_id, stamp)
        self.map_name = map_name
