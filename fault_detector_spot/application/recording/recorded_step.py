"""Recording-only playback policy, separate from command execution parameters."""

from dataclasses import dataclass

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.commanding.semantic_command import SemanticCommand


RETRYABLE_COMMANDS = frozenset({
    CommandID.MOVE_TO_WAYPOINT,
    CommandID.MOVE_ARM_TO_TAG,
    CommandID.FOLLOW_MOVE_TO_TAG_PATH,
    CommandID.WAIT_TIME,
})


def validate_retries(value):
    if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 100:
        raise ValueError("Playback retries must be an integer between 0 and 100")
    return value


@dataclass(frozen=True)
class RecordedStep:
    command: SemanticCommand
    playback_retries: int = 0

    def __post_init__(self):
        if not isinstance(self.command, SemanticCommand):
            raise TypeError("Recorded step requires a semantic command")
        validate_retries(self.playback_retries)
        if self.playback_retries and self.command.command_id not in RETRYABLE_COMMANDS:
            raise ValueError(f"Playback retry is unsupported for {self.command.command_id.value}")
