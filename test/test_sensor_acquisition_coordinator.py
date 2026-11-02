"""Essential coordinator lifecycle tests."""

from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest
from builtin_interfaces.msg import Time
from fault_detector_msgs.msg import SensorAcquisitionState as StateMessage
from nav_msgs.msg import Odometry
from sensor_msgs.msg import MagneticField

from fault_detector_spot.application.api.sensor_acquisition_api import (
    SensorAcquisitionApi,
)
from fault_detector_spot.application.coordinators.sensor_acquisition_coordinator import (
    SensorAcquisitionCoordinator,
    SensorAcquisitionRequest,
    SensorAcquisitionStatus,
)
from fault_detector_spot.inspection.measurement import (
    MeasurementCompletionState,
    MeasurementRepository,
)
from fault_detector_spot.shared.geometry.models import (
    PoseData,
    QuaternionData,
    Vector3Data,
)
from fault_detector_spot.inspection.model.sensor_models import (
    SensorChannel,
    SensorDefinition,
)


START_NS = 1_788_371_731_438_271_923


def pose(x=0.0):
    return PoseData(
        position=Vector3Data(x=x, y=0.0, z=0.0),
        orientation=QuaternionData.identity(),
    )


def definition():
    return SensorDefinition(
        sensor_id="bmm150_probe",
        display_name="BMM150 probe",
        hand_to_probe=pose(),
        channels=(
            SensorChannel(
                channel_id="magnetic_field",
                topic="/sensors/bmm150_probe/magnetic_field",
                message_type="sensor_msgs/msg/MagneticField",
            ),
            SensorChannel(
                channel_id="odometry",
                topic="/odom",
                message_type="nav_msgs/msg/Odometry",
            ),
        ),
    )


def make_coordinator(tmp_path, service_ready=True):
    node = MagicMock()
    now = MagicMock(nanoseconds=START_NS)
    now.to_msg.return_value = Time()
    node.get_clock.return_value.now.return_value = now

    client = MagicMock()
    client.service_is_ready.return_value = service_ready
    start_future = MagicMock()
    stop_future = MagicMock()
    client.call_async.side_effect = [start_future, stop_future]
    node.create_client.return_value = client

    attachment = SimpleNamespace(
        sensor_id="bmm150_probe",
        attachment_revision=4,
        has_sensor=True,
    )
    attachments = MagicMock()
    attachments.require_motion_attachment.return_value = attachment
    sensors = MagicMock()
    sensors.load.return_value = definition()
    repository = MeasurementRepository(tmp_path)
    coordinator = SensorAcquisitionCoordinator(
        node=node,
        measurement_repository=repository,
        sensor_attachment_controller=attachments,
        sensor_repository=sensors,
    )
    return coordinator, repository, node, client, start_future, stop_future


def request():
    return SensorAcquisitionRequest(
        object_id="motor_01",
        routine_id="magnetic_scan",
        probe_point_id="bearing_front",
        object_pose_execution=pose(10.0),
    )


def complete(future, success=True):
    future.result.return_value = SimpleNamespace(
        success=success,
        detail="ok",
    )
    future.add_done_callback.call_args.args[0](future)


def test_physical_sample_controls_readiness_and_stop(tmp_path):
    coordinator, repository, node, client, started, stopped = (
        make_coordinator(tmp_path)
    )
    publisher = MagicMock()
    node.create_publisher.return_value = publisher
    api = SensorAcquisitionApi(node, coordinator)

    assert coordinator.start(request()).status is SensorAcquisitionStatus.STARTING
    callbacks = {
        call.args[1]: call.args[2]
        for call in node.create_subscription.call_args_list
    }
    callbacks["/odom"](Odometry())
    complete(started)
    assert coordinator.snapshot().status is SensorAcquisitionStatus.STARTING

    callbacks["/sensors/bmm150_probe/magnetic_field"](MagneticField())
    assert coordinator.snapshot().status is SensorAcquisitionStatus.RECORDING
    assert coordinator.stop().status is SensorAcquisitionStatus.STOPPING
    complete(stopped)
    assert coordinator.snapshot().status is SensorAcquisitionStatus.IDLE
    assert [call.args[0].enabled for call in client.call_async.call_args_list] == [
        True,
        False,
    ]

    saved = repository.load(
        object_id="motor_01",
        routine_id="magnetic_scan",
        probe_point_id="bearing_front",
        sensor_id="bmm150_probe",
        started_at_ns=START_NS,
    )
    assert saved.completion_state is MeasurementCompletionState.COMPLETE
    states = [call.args[0].state for call in publisher.publish.call_args_list]
    assert StateMessage.STATE_RECORDING in states
    assert states[-1] == StateMessage.STATE_IDLE
    api.close()


def test_offline_head_skips_without_creating_files(tmp_path):
    coordinator, _repository, node, _client, _started, _stopped = (
        make_coordinator(tmp_path, service_ready=False)
    )

    state = coordinator.start(request())

    assert state.status is SensorAcquisitionStatus.IDLE
    assert "offline" in state.detail
    assert not tuple(tmp_path.rglob("metadata.json"))
    node.destroy_client.assert_not_called()


def test_latched_software_stop_blocks_every_coordinator_start_entry(tmp_path):
    coordinator, _, _, client, _, _ = make_coordinator(tmp_path)
    coordinator.latch_software_stop()
    with pytest.raises(RuntimeError, match="Software stop is latched"):
        coordinator.start(request())
    assert not client.call_async.called
    assert not tuple(tmp_path.rglob("metadata.json"))
    coordinator.reset_software_stop()
    assert coordinator.start(request()).status is SensorAcquisitionStatus.STARTING


def test_software_stop_during_preparation_prevents_sensor_start_and_retains_cancelled_metadata(
    tmp_path, monkeypatch,
):
    coordinator, repository, _, client, _, _ = make_coordinator(tmp_path)
    create_sources = coordinator._create_sources

    def stop_during_preparation(*args):
        create_sources(*args)
        coordinator.latch_software_stop()

    monkeypatch.setattr(coordinator, "_create_sources", stop_during_preparation)
    coordinator.start(request())

    assert coordinator.recording_stopped
    assert coordinator.software_stop_latched
    assert not client.call_async.called
    saved = repository.load(
        object_id="motor_01", routine_id="magnetic_scan", probe_point_id="bearing_front",
        sensor_id="bmm150_probe", started_at_ns=START_NS,
    )
    assert saved.completion_state is MeasurementCompletionState.CANCELLED
    assert saved.sample_counts["magnetic_field"] == 0


@pytest.mark.parametrize("ordinary_stop_started", [False, True])
def test_latch_alone_preserves_cancelled_metadata_when_head_ack_overtakes_abort_callback(
    tmp_path, ordinary_stop_started,
):
    coordinator, repository, _, _, started, stopped = make_coordinator(tmp_path)
    coordinator.start(request())
    complete(started)
    coordinator._session.sources[0]._receive_message(MagneticField())
    if ordinary_stop_started:
        coordinator.stop()
    coordinator.latch_software_stop()
    if not ordinary_stop_started:
        coordinator.stop()  # A timer can request ordinary completion before the abort callback runs.
    complete(stopped)
    saved = repository.load(
        object_id="motor_01", routine_id="magnetic_scan", probe_point_id="bearing_front",
        sensor_id="bmm150_probe", started_at_ns=START_NS,
    )
    assert saved.completion_state is MeasurementCompletionState.CANCELLED
    assert saved.sample_counts["magnetic_field"] == 1


def test_sample_write_failure_is_preserved_through_source_and_stop(tmp_path, monkeypatch):
    coordinator, repository, node, client, started, stopped = make_coordinator(tmp_path)
    coordinator.start(request())
    complete(started)
    callbacks = {
        call.args[1]: call.args[2]
        for call in node.create_subscription.call_args_list
    }
    receive = callbacks["/sensors/bmm150_probe/magnetic_field"]
    receive(MagneticField())
    assert coordinator.snapshot().status is SensorAcquisitionStatus.RECORDING
    active = coordinator._session.recording
    stream = repository._open_recordings[active.identity].channel_files["magnetic_field"]

    def fail_write(_line):
        raise OSError("disk full")

    with monkeypatch.context() as patch:
        patch.setattr(stream, "write", fail_write)
        receive(MagneticField())
    receive(MagneticField())
    coordinator.stop()
    complete(stopped)

    assert coordinator.snapshot().status is SensorAcquisitionStatus.FAILED
    assert "samples could not be saved" in coordinator.snapshot().detail
    saved = repository.load(
        object_id="motor_01", routine_id="magnetic_scan",
        probe_point_id="bearing_front", sensor_id="bmm150_probe",
        started_at_ns=START_NS,
    )
    assert saved.completion_state is MeasurementCompletionState.FAILED
    assert saved.sample_counts["magnetic_field"] == 2
    assert not repository.is_open(active)
    assert coordinator._session is None


def test_failed_attempt_is_finalized_and_retry_creates_separate_recording(tmp_path):
    coordinator, repository, node, client, started, stopped = make_coordinator(tmp_path)
    assert coordinator.recording_stopped
    coordinator.start(request())
    complete(started)
    first_sample = coordinator._session.sources[0]._on_first_sample
    coordinator._session.sources[0]._receive_message(MagneticField())
    assert coordinator.snapshot().status is SensorAcquisitionStatus.RECORDING
    assert not coordinator.recording_stopped
    coordinator.abort_recording()
    assert not coordinator.recording_stopped
    complete(stopped)
    assert coordinator.recording_stopped
    assert coordinator.snapshot().status is SensorAcquisitionStatus.FAILED
    failed = repository.load(object_id="motor_01", routine_id="magnetic_scan",
                             probe_point_id="bearing_front", sensor_id="bmm150_probe",
                             started_at_ns=START_NS)
    assert failed.completion_state is MeasurementCompletionState.FAILED
    next_started, next_stopped = MagicMock(), MagicMock()
    client.call_async.side_effect = [next_started, next_stopped]
    coordinator.start(request())
    complete(next_started)
    first_sample("magnetic_field", START_NS)  # Late callback from the failed attempt.
    assert coordinator.snapshot().status is SensorAcquisitionStatus.STARTING
    coordinator._session.sources[0]._receive_message(MagneticField())
    assert coordinator.snapshot().status is SensorAcquisitionStatus.RECORDING
    assert coordinator._session.recording.started_at_ns == START_NS + 1
    coordinator.stop()
    complete(next_stopped)
    assert coordinator.recording_stopped
    assert repository.load(object_id="motor_01", routine_id="magnetic_scan",
                           probe_point_id="bearing_front", sensor_id="bmm150_probe",
                           started_at_ns=START_NS + 1).completion_state is MeasurementCompletionState.COMPLETE
    assert repository.load(object_id="motor_01", routine_id="magnetic_scan",
                           probe_point_id="bearing_front", sensor_id="bmm150_probe",
                           started_at_ns=START_NS).completion_state is MeasurementCompletionState.FAILED


def test_failed_sensor_stop_cannot_be_cleared_to_allow_a_retry(tmp_path):
    import pytest
    coordinator, _, _, _, started, stopped = make_coordinator(tmp_path)
    coordinator.start(request())
    complete(started)
    coordinator.abort_recording()
    complete(stopped, success=False)
    assert not coordinator.recording_stopped
    assert coordinator.stop().status is SensorAcquisitionStatus.FAILED
    with pytest.raises(RuntimeError, match="stop is unconfirmed"):
        coordinator.start(request())


@pytest.mark.parametrize("phase", ["starting", "recording", "stopping", "failed"])
def test_software_stop_retains_interrupted_measurement_and_requests_shutdown_once(tmp_path, phase):
    coordinator, repository, _, client, started, stopped = make_coordinator(tmp_path)
    coordinator.start(request())
    if phase != "starting":
        complete(started)
        coordinator._session.sources[0]._receive_message(MagneticField())
    if phase == "stopping":
        coordinator.stop()
    elif phase == "failed":
        coordinator.abort_recording()

    coordinator.stop(cancelled=True)
    coordinator.stop(cancelled=True)
    assert not coordinator.recording_stopped
    assert [call.args[0].enabled for call in client.call_async.call_args_list] == [True, False]
    if phase == "starting":
        refreshed_stop = MagicMock()
        client.call_async.side_effect = [refreshed_stop]
        complete(started)
        complete(stopped)  # Earlier stop response does not cover the late start.
        assert not coordinator.recording_stopped
        stopped = refreshed_stop
    complete(stopped)

    saved = repository.load(
        object_id="motor_01", routine_id="magnetic_scan",
        probe_point_id="bearing_front", sensor_id="bmm150_probe", started_at_ns=START_NS,
    )
    assert saved.completion_state is (
        MeasurementCompletionState.FAILED if phase == "failed"
        else MeasurementCompletionState.CANCELLED
    )
    assert saved.sample_counts["magnetic_field"] == (0 if phase == "starting" else 1)
    assert coordinator.recording_stopped


@pytest.mark.parametrize("ros_nanoseconds", [START_NS, 0, START_NS * 2])
def test_software_stop_timeout_uses_monotonic_time_and_retains_failed_metadata(
    tmp_path, monkeypatch, ros_nanoseconds,
):
    from rclpy.clock import ClockType
    from fault_detector_spot.application.coordinators import sensor_acquisition_coordinator as module

    now = [100.0]
    monkeypatch.setattr(module.time, "monotonic", lambda: now[0])
    coordinator, repository, node, _, _, _ = make_coordinator(tmp_path)
    assert node.create_timer.call_args.kwargs["clock"].clock_type == ClockType.STEADY_TIME
    coordinator.start(request())
    coordinator.stop(cancelled=True)
    node.get_clock.return_value.now.return_value.nanoseconds = ros_nanoseconds
    now[0] += coordinator.STOP_TIMEOUT_SEC
    coordinator._tick()

    assert not coordinator.recording_stopped
    assert coordinator.snapshot().status is SensorAcquisitionStatus.FAILED
    assert "Timed out" in coordinator.snapshot().detail
    saved = repository.load(
        object_id="motor_01", routine_id="magnetic_scan",
        probe_point_id="bearing_front", sensor_id="bmm150_probe", started_at_ns=START_NS,
    )
    assert saved.completion_state is MeasurementCompletionState.FAILED


def test_timeout_timer_survives_finish_and_is_reused(tmp_path):
    coordinator, repository, node, client, started, stopped = make_coordinator(tmp_path)
    timer = node.create_timer.return_value
    callback = node.create_timer.call_args.args[1]
    coordinator.start(request())
    complete(started)
    coordinator.stop()
    complete(stopped)
    timer.cancel.assert_called()
    node.destroy_timer.assert_not_called()
    callback()  # Executor may already have taken a callback before cancellation.
    assert coordinator.snapshot().status is SensorAcquisitionStatus.IDLE
    client.call_async.side_effect = None
    coordinator.start(request())
    assert node.create_timer.call_count == 1
    assert timer.reset.call_count == 2
    coordinator.request_shutdown()
    coordinator.close(stop_unconfirmed=True)
    callback()
    node.destroy_timer.assert_not_called()


def test_recording_resources_reused_and_idle_messages_ignored(tmp_path):
    coordinator, repository, node, client, started, stopped = make_coordinator(tmp_path)
    for attempt in range(3):
        started, stopped = MagicMock(), MagicMock()
        client.call_async.side_effect = [started, stopped]
        coordinator.start(request())
        complete(started)
        receive = node.create_subscription.call_args_list[0].args[2]
        receive(MagneticField())
        recording = coordinator._session.recording
        assert coordinator.snapshot().status is SensorAcquisitionStatus.RECORDING
        coordinator.stop()
        complete(stopped)
        receive(MagneticField())  # Already queued when finalization stopped the source.
        saved = repository.load(
            object_id=recording.object_id, routine_id=recording.routine_id,
            probe_point_id=recording.probe_point_id, sensor_id=recording.sensor_id,
            started_at_ns=recording.started_at_ns,
        )
        assert saved.sample_counts['magnetic_field'] == 1
    assert node.create_subscription.call_count == 2
    assert node.create_client.call_count == 1
    node.destroy_subscription.assert_not_called()
    node.destroy_client.assert_not_called()


def test_geometry_tf_listener_survives_recording_finalization(tmp_path, monkeypatch):
    from fault_detector_spot.inspection.model.sensor_models import SensorChannelSource
    coordinator, _, node, client, _, _ = make_coordinator(tmp_path)
    definition_with_geometry = definition()
    from dataclasses import replace
    coordinator.sensors.load.return_value = replace(
        definition_with_geometry,
        channels=definition_with_geometry.channels + (SensorChannel(
            channel_id='geometry', topic='', message_type='',
            source_kind=SensorChannelSource.SPOT_GEOMETRY,
        ),),
    )
    listener = MagicMock()
    buffer_factory = MagicMock()
    listener_factory = MagicMock(return_value=listener)
    module = 'fault_detector_spot.application.coordinators.sensor_acquisition_coordinator.tf2_ros'
    monkeypatch.setattr(module + '.Buffer', buffer_factory)
    monkeypatch.setattr(module + '.TransformListener', listener_factory)
    for _ in range(2):
        started, stopped = MagicMock(), MagicMock()
        client.call_async.side_effect = [started, stopped]
        coordinator.start(request())
        complete(started)
        coordinator.stop()
        complete(stopped)
    listener_factory.assert_called_once()
    buffer_factory.assert_called_once()
    listener.unregister.assert_not_called()
    node.destroy_subscription.assert_not_called()


def test_shutdown_waits_for_delayed_start_and_a_fresh_stop_reply(tmp_path):
    coordinator, _, node, client, started, stopped = make_coordinator(tmp_path)
    refreshed = MagicMock()
    client.call_async.side_effect = [started, stopped, refreshed]
    coordinator.start(request())
    coordinator.request_shutdown()
    complete(stopped)
    assert not coordinator.shutdown_status()[0]
    node.destroy_client.assert_not_called()
    node.destroy_subscription.assert_not_called()
    complete(started)
    assert not coordinator.shutdown_status()[0]
    complete(refreshed)
    assert coordinator.shutdown_status()[0]
    coordinator.close()
    assert [call.args[0].enabled for call in client.call_async.call_args_list] == [True, False, False]


def test_shutdown_close_needs_confirmation_or_explicit_timeout(tmp_path):
    coordinator, repository, node, _, started, _ = make_coordinator(tmp_path)
    coordinator.start(request())
    complete(started)
    coordinator.request_shutdown()
    with pytest.raises(RuntimeError, match='confirmation'):
        coordinator.close()
    assert coordinator._session is not None
    coordinator.close(stop_unconfirmed=True)
    assert not coordinator.recording_stopped
    assert 'stop-unconfirmed' in coordinator.snapshot().detail
    assert repository.load(object_id='motor_01', routine_id='magnetic_scan',
                           probe_point_id='bearing_front', sensor_id='bmm150_probe',
                           started_at_ns=START_NS).completion_state is MeasurementCompletionState.FAILED
    coordinator.close(stop_unconfirmed=True)


def test_shutdown_recording_finalization_can_retry_after_a_cleanup_error(tmp_path, monkeypatch):
    coordinator, repository, _, _, started, _ = make_coordinator(tmp_path)
    coordinator.start(request())
    complete(started)
    coordinator.request_shutdown()
    finalize = repository.finalize
    failing = MagicMock(side_effect=OSError('disk unavailable'))
    monkeypatch.setattr(repository, 'finalize', failing)
    with pytest.raises(OSError, match='disk unavailable'):
        coordinator.close(stop_unconfirmed=True)
    assert coordinator._session is not None
    monkeypatch.setattr(repository, 'finalize', finalize)
    coordinator.close(stop_unconfirmed=True)
    assert coordinator._session is None
    assert not coordinator.recording_stopped


def test_recording_source_cleanup_error_does_not_block_head_stop_or_other_sources(tmp_path):
    coordinator, _, _, client, started, stopped = make_coordinator(tmp_path)
    coordinator.start(request())
    complete(started)
    source = MagicMock()
    other = MagicMock()
    source.stop.side_effect = [RuntimeError('source busy'), RuntimeError('source busy'), None]
    coordinator._session.sources = [source, other]
    coordinator.request_shutdown()
    assert not client.call_async.call_args.args[0].enabled
    other.stop.assert_called_once()
    with pytest.raises(RuntimeError, match='source busy'):
        complete(stopped)
    assert coordinator._session is not None
    coordinator.close(stop_unconfirmed=True)
    assert coordinator._session is None
    assert source.stop.call_count == 3
    other.stop.assert_called_once()


def test_normal_finalization_failure_remains_owned_until_shutdown_retry(tmp_path, monkeypatch):
    from fault_detector_spot.shared.persistence import file_storage

    coordinator, repository, _, _, started, stopped = make_coordinator(tmp_path)
    coordinator.start(request())
    complete(started)
    active = coordinator._session.recording
    path = repository.get_metadata_path(active)
    original = path.read_bytes()

    def fail_replace(*_args):
        raise OSError('disk unavailable')

    coordinator.stop()
    with monkeypatch.context() as patch:
        patch.setattr(file_storage.os, 'replace', fail_replace)
        with pytest.raises(OSError, match='disk unavailable'):
            complete(stopped)
    assert coordinator._session is not None
    assert repository.is_open(active)
    assert path.read_bytes() == original
    assert coordinator.snapshot().status is SensorAcquisitionStatus.FAILED

    coordinator.request_shutdown()
    assert coordinator.shutdown_status()[0]
    coordinator.close()
    assert not repository.is_open(active)
    assert repository.load(object_id='motor_01', routine_id='magnetic_scan',
                           probe_point_id='bearing_front', sensor_id='bmm150_probe',
                           started_at_ns=START_NS).completion_state is MeasurementCompletionState.FAILED
