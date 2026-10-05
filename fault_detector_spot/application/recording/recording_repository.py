"""Persist semantic command recordings without owning playback or ROS state."""

import json
from pathlib import Path
from typing import Iterable

from fault_detector_spot.application.commanding.semantic_command import (
    SemanticCommand,
)
from fault_detector_spot.application.recording.semantic_command_codec import (
    deserialize_recording,
    serialize_recorded_command,
)
from fault_detector_spot.shared.persistence.file_storage import (
    atomic_write_text,
    validate_storage_name,
)


class RecordingRepository:
    """Store named command sequences in the existing JSON recording format."""

    def __init__(self, root: str | Path) -> None:
        """Create the recording directory if it does not exist."""
        self.root = Path(root).expanduser()
        self.root.mkdir(parents=True, exist_ok=True)

    def path(self, name: str) -> Path:
        """Resolve a name, rejecting invalid names and symbolic links."""
        validate_storage_name(name, 'recording name')
        path = self.root / f'{name}.json'
        if path.is_symlink():
            raise ValueError('Recording files must not be symbolic links')
        return path

    def save(self, name: str, commands: Iterable[SemanticCommand]) -> None:
        """Atomically replace a recording with the supplied commands."""
        path = self.path(name)
        document = {
            'commands': [
                serialize_recorded_command(command) for command in commands
            ],
        }
        content = json.dumps(document, indent=2) + '\n'
        atomic_write_text(path, content)

    def load(self, name: str) -> list[SemanticCommand]:
        """Read and validate a recording; propagate read and format errors."""
        content = self.path(name).read_text(encoding='utf-8')
        document = json.loads(content)
        return deserialize_recording(document)

    def delete(self, name: str) -> bool:
        """Delete a recording and return whether a file was removed."""
        path = self.path(name)
        if not path.exists():
            return False
        path.unlink()
        return True

    def list_names(self) -> list[str]:
        """Return sorted recording names, excluding invalid names and links."""
        names = []
        for path in self.root.iterdir():
            if path.suffix != '.json':
                continue
            try:
                if self.path(path.stem).is_file():
                    names.append(path.stem)
            except (TypeError, ValueError):
                continue
        return sorted(names)
