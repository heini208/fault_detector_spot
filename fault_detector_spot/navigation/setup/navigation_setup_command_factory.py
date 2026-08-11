"""Build internal commands for navigation runtime setup operations."""

from fault_detector_msgs.msg import ComplexCommand

from fault_detector_spot.application.commanding.command_ids import CommandID


class NavigationSetupCommandFactory:
    """Construct navigation setup commands without workflow state."""

    _SUPPORTED = frozenset({
        CommandID.START_SLAM,
        CommandID.START_LOCALIZATION,
        CommandID.STOP_MAPPING,
        CommandID.SWAP_MAP,
    })

    def create(
        self,
        command_id: CommandID,
        map_name: str = "",
    ) -> ComplexCommand:
        """Return one validated internal navigation setup command."""
        try:
            normalized_id = CommandID(command_id)
        except (TypeError, ValueError) as exception:
            raise ValueError(
                f"Unsupported navigation setup command: {command_id!r}"
            ) from exception
        if normalized_id not in self._SUPPORTED:
            raise ValueError(
                f"Unsupported navigation setup command: {normalized_id.value}"
            )
        normalized_map = map_name.strip()
        if normalized_id == CommandID.SWAP_MAP and not normalized_map:
            raise ValueError("Map name must not be empty")
        command = ComplexCommand()
        command.command.command_id = normalized_id.value
        command.map_name = normalized_map
        return command


__all__ = ["NavigationSetupCommandFactory"]
