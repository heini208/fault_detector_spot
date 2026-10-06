"""Essential coordinator lifecycle tests."""

from types import SimpleNamespace
from unittest.mock import MagicMock

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
    coordinator.close()
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
