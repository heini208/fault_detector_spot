"""Persistence owns recording encoding and preserves files on failed writes."""

import json
from pathlib import Path

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.commanding.semantic_command import (
    CommandQuaternion, CommandVector3, SemanticCommand, SemanticTag, StampedPose,
)
from fault_detector_spot.application.recording.recording_repository import (
    RecordingRepository,
)
from fault_detector_spot.shared.persistence import file_storage
from fault_detector_spot.application.recording.semantic_command_codec import deserialize_recording

import pytest


def test_repository_round_trip_writes_current_recording_version(tmp_path):
    repository = RecordingRepository(tmp_path)
    command = SemanticCommand(command_id=CommandID.STAND_UP)
    repository.save('run', [command])
    document = json.loads(repository.path('run').read_text())
    assert document['schema_version'] == 1
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


def test_playback_retries_are_separate_from_execution_retries(tmp_path):
    from fault_detector_spot.application.recording.recorded_step import RecordedStep
    repository = RecordingRepository(tmp_path)
    command = SemanticCommand(command_id=CommandID.MOVE_TO_WAYPOINT, retries=5)
    repository.save('run', [RecordedStep(command, 3)])
    assert repository.load_steps('run') == [RecordedStep(command, 3)]
    assert repository.load('run')[0].retries == 5
    repository.save('old', [command])
    assert repository.load_steps('old')[0].playback_retries == 0


@pytest.mark.parametrize('count', [-1, 101, True, 1.5, '2'])
def test_invalid_playback_retry_metadata_is_rejected(tmp_path, count):
    repository = RecordingRepository(tmp_path)
    repository.save('run', [SemanticCommand(command_id=CommandID.MOVE_TO_WAYPOINT)])
    data = json.loads(repository.path('run').read_text())
    data['commands'][0]['playback_retries'] = count
    repository.path('run').write_text(json.dumps(data))
    with pytest.raises(ValueError):
        repository.load_steps('run')


def test_skip_block_round_trip_and_old_recording_default(tmp_path):
    from fault_detector_spot.application.recording.recorded_step import RecordedStep
    repository = RecordingRepository(tmp_path)
    command = SemanticCommand(command_id=CommandID.MOVE_TO_WAYPOINT)
    steps = [RecordedStep(command, 2, 1), RecordedStep(command, 0, 2), RecordedStep(command)]
    repository.save('run', steps)
    assert repository.load_steps('run') == steps
    repository.save('old', [command])
    assert repository.load_steps('old')[0].skip_block == 0


@pytest.mark.parametrize('blocks', [[1, 0, 1], [1, 2, 1], [-1], [True], ['1'], [1.5]])
def test_invalid_skip_boundaries_are_rejected(tmp_path, blocks):
    repository = RecordingRepository(tmp_path)
    command = SemanticCommand(command_id=CommandID.MOVE_TO_WAYPOINT)
    repository.save('run', [command] * len(blocks))
    data = json.loads(repository.path('run').read_text())
    for entry, block in zip(data['commands'], blocks):
        entry['skip_block'] = block
    repository.path('run').write_text(json.dumps(data))
    with pytest.raises(ValueError):
        repository.load_steps('run')


@pytest.mark.parametrize('version', [0, 1])
@pytest.mark.parametrize('policy', [{}, {'playback_retries': 3, 'skip_block': 7}])
def test_version_fixture_preserves_command_meaning_without_rewriting_on_load(tmp_path, version, policy):
    content = (Path(__file__).parent / 'fixtures' / 'recordings' / f'v{version}.json').read_bytes()
    if policy:
        document = json.loads(content)
        document['commands'][0].update(policy)
        content = json.dumps(document).encode()
    repository = RecordingRepository(tmp_path)
    source = repository.path('original')
    source.write_bytes(content)
    modified = source.stat().st_mtime_ns
    expected = SemanticCommand(
        CommandID.FOLLOW_MOVE_TO_TAG_PATH,
        tag=SemanticTag(7, StampedPose(
            frame_id='odom', position=CommandVector3(1.0, 2.0, 0.5),
            orientation=CommandQuaternion(w=1.0),
        )),
        offset=StampedPose(
            frame_id='tag_7', position=CommandVector3(0.1, 0.0, 0.0),
            orientation=CommandQuaternion(w=1.0),
        ),
        pre_approach_offsets=(StampedPose(
            frame_id='tag_7', position=CommandVector3(0.2, 0.0, 0.0),
            orientation=CommandQuaternion(w=1.0),
        ),),
        wait_time=1.25, orientation_mode='relative_to_tag', arm_speed_scale=0.5,
    )
    steps = repository.load_steps('original')
    assert steps[0].command == expected
    assert steps[0].playback_retries == policy.get('playback_retries', 0)
    assert steps[0].skip_block == policy.get('skip_block', 0)
    assert deserialize_recording(json.loads(content)) == [expected]
    assert source.read_bytes() == content
    assert source.stat().st_mtime_ns == modified
    repository.save('round_trip', steps)
    assert repository.load_steps('round_trip') == steps
    assert json.loads(repository.path('round_trip').read_text())['schema_version'] == 1


@pytest.mark.parametrize('version', [True, 1.0, 1.5, '1', None, 2, -1])
def test_invalid_or_unsupported_recording_version_is_rejected(tmp_path, version):
    repository = RecordingRepository(tmp_path)
    repository.path('run').write_text(json.dumps({'schema_version': version, 'commands': []}))
    with pytest.raises(ValueError, match='schema_version'):
        repository.load('run')


def test_current_version_does_not_apply_legacy_defaults_or_command_renames(tmp_path):
    repository = RecordingRepository(tmp_path)
    legacy = json.loads((Path(__file__).parent / 'fixtures' / 'recordings' / 'v0.json').read_text())
    legacy['schema_version'] = 0
    repository.path('run').write_text(json.dumps(legacy))
    assert repository.load('run')[0].command_id == CommandID.FOLLOW_MOVE_TO_TAG_PATH
    legacy['schema_version'] = 1
    repository.path('run').write_text(json.dumps(legacy))
    with pytest.raises(ValueError, match='command 1:.*missing field.*retries'):
        repository.load('run')
    current = json.loads((Path(__file__).parent / 'fixtures' / 'recordings' / 'v1.json').read_text())
    current['commands'][0]['command_id'] = CommandID.MOVE_ARM_TO_TAG.value
    repository.path('run').write_text(json.dumps(current))
    with pytest.raises(ValueError, match='command 1:.*Paths require'):
        repository.load('run')


@pytest.mark.parametrize(('field', 'value'), [
    ('wait_time', float('nan')), ('wait_time', float('inf')), ('wait_time', -float('inf')),
    ('wait_time', True), ('wait_time', '1.0'), ('map_name', None), ('map_name', 4),
    ('retries', True), ('retries', 1.5), ('retries', '1'),
])
def test_recording_scalars_are_validated_before_conversion(tmp_path, field, value):
    repository = RecordingRepository(tmp_path)
    repository.save('run', [SemanticCommand(CommandID.STAND_UP)])
    document = json.loads(repository.path('run').read_text())
    document['commands'][0][field] = value
    repository.path('run').write_text(json.dumps(document))
    with pytest.raises(ValueError, match=f'command 1:.*{field}'):
        repository.load('run')


@pytest.mark.parametrize('tag_id', [True, 1.0, 1.5, '7'])
def test_recording_tag_ids_reject_coercion(tag_id):
    document = json.loads((Path(__file__).parent / 'fixtures' / 'recordings' / 'v1.json').read_text())
    document['commands'][0]['tag']['id'] = tag_id
    with pytest.raises(ValueError, match='command 1:.*tag'):
        deserialize_recording(document)


def test_save_rejects_unreadable_scalar_without_replacing_prior_recording(tmp_path):
    repository = RecordingRepository(tmp_path)
    original = SemanticCommand(CommandID.STAND_UP)
    repository.save('run', [original])
    malformed = SemanticCommand(CommandID.STAND_UP, arm_speed_scale=True)
    with pytest.raises(TypeError, match='arm_speed_scale must be a number'):
        repository.save('run', [malformed])
    assert repository.load('run') == [original]
