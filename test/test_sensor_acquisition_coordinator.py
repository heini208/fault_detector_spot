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
from fault_detector_spot.inspection.model.models import (
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
    node.destroy_client.assert_called_once()
