"""Persistence owns recording encoding and preserves files on failed writes."""

import json

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.commanding.semantic_command import (
    SemanticCommand,
)
from fault_detector_spot.application.recording.recording_repository import (
    RecordingRepository,
)
from fault_detector_spot.shared.persistence import file_storage

import pytest


def test_repository_round_trip_preserves_existing_recording_format(tmp_path):
    """Preserve the JSON format across repository operations."""
    repository = RecordingRepository(tmp_path)
    command = SemanticCommand(command_id=CommandID.STAND_UP)
    repository.save('run', [command])
    document = json.loads(repository.path('run').read_text())
    assert document['commands'][0]['command_id'] == 'stand_up'
    assert repository.load('run') == [command]
    assert repository.list_names() == ['run']
    assert repository.delete('run')
    assert not repository.delete('run')
    assert repository.list_names() == []


def test_failed_replacement_preserves_previous_recording(
    tmp_path, monkeypatch,
):
    """Keep the previous file and clean up after a failed save."""
    repository = RecordingRepository(tmp_path)
    original = SemanticCommand(command_id=CommandID.STAND_UP)
    repository.save('run', [original])

    def fail_replace(*_args):
        raise OSError('disk failure')

    monkeypatch.setattr(file_storage.os, 'replace', fail_replace)
    replacement = SemanticCommand(command_id=CommandID.SIT_DOWN)
    with pytest.raises(OSError, match='disk failure'):
        repository.save('run', [replacement])
    assert repository.load('run') == [original]
    assert sorted(path.name for path in tmp_path.iterdir()) == ['run.json']
