"""Build semantic commands for navigation runtime setup operations."""

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.commanding.semantic_command import (
    SemanticCommand,
)


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
    ) -> SemanticCommand:
        try:
            normalized_id = CommandID(command_id)
        except (TypeError, ValueError) as exception:
            raise ValueError(
                "Unsupported navigation setup command: "
                f"{command_id!r}"
            ) from exception
        if normalized_id not in self._SUPPORTED:
            raise ValueError(
                "Unsupported navigation setup command: "
                f"{normalized_id.value}"
            )
        normalized_map = map_name.strip()
        if (
            normalized_id == CommandID.SWAP_MAP
            and not normalized_map
        ):
            raise ValueError(
                "Map name must not be empty"
            )
        return SemanticCommand(
            command_id=normalized_id,
            map_name=normalized_map,
        )


__all__ = ["NavigationSetupCommandFactory"]
