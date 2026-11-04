"""Tests for application-owned sensor registry mutations."""

from dataclasses import replace
from threading import RLock

import pytest
import yaml
from fault_detector_msgs.srv import AddSensor, DeleteSensor, UpdateSensor

from fault_detector_spot.application.api.sensor_registry_api import (
    SensorRegistryApi,
)
from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.commanding.command_request import (
    CommandOrigin,
    CommandRequest,
    RecordingPolicy,
)
from fault_detector_spot.application.commanding.semantic_command import (
    SemanticCommand,
)
from fault_detector_spot.application.controllers.command_controller import (
    CommandController,
    CommandControllerState,
    CommandExecutionStatus,
    SoftwareStopLatched,
)
from fault_detector_spot.application.controllers.sensor_attachment_controller import (
    SensorAttachmentController,
)
from fault_detector_spot.application.controllers.sensor_registry_controller import (
    SensorRegistryController,
)
from fault_detector_spot.inspection.model.sensor_models import (
    DEFAULT_HAND_SENSOR_ID,
    SensorChannel,
    SensorChannelSource,
    sensor_definition_from_values,
)
from fault_detector_spot.inspection.repository.sensor_attachment_state_store import (
    SensorAttachmentStateStore,
)
from fault_detector_spot.inspection.repository.sensor_repository import (
    SensorRepository,
)


class FakeBroadcaster:
    """Capture static transforms sent by the registry API."""

    def __init__(self):
        self.transforms = []

    def sendTransform(self, transform):
        self.transforms.append(transform)


class FakeLogger:
    """Provide the logger surface used by service callbacks."""

    def info(self, message):
        return None

    def error(self, message):
        return None


class FakeNode:
    """Provide the node surface used by callback diagnostics."""

    def __init__(self):
        self.logger = FakeLogger()

    def get_logger(self):
        return self.logger


def definition(sensor_id="test", x=0.20):
    """Create one valid physical sensor definition."""
    return sensor_definition_from_values(
        sensor_id,
        "Test sensor",
        x,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )


def controllers(tmp_path):
    """Create one shared repository and application authority."""
    repository = SensorRepository(tmp_path / "sensors")
    command_controller = CommandController()
    attachment_controller = SensorAttachmentController(
        repository,
        SensorAttachmentStateStore(tmp_path / "attachment.yaml"),
        command_controller,
    )
    registry_controller = SensorRegistryController(
        repository,
        attachment_controller,
        command_controller,
    )
    return (
        registry_controller,
        attachment_controller,
        command_controller,
        repository,
    )


def api_state(tmp_path):
    """Build the non-ROS state needed by registry API callbacks."""
    registry, attachment, commands, repository = controllers(tmp_path)
    api = SensorRegistryApi.__new__(SensorRegistryApi)
    api.node = FakeNode()
    api.controller = registry
    api._lock = RLock()
    api._static_broadcaster = FakeBroadcaster()
    api.published = []
    api._transform_message = lambda value: value
    api._publish_definitions = (
        lambda values: api.published.append(tuple(values))
    )
    return api, attachment, commands, repository


def add_request(sensor_id="test", x=0.20):
    """Create one manual hand-to-probe transform request."""
    value = AddSensor.Request()
    value.sensor.sensor_id = sensor_id
    value.sensor.display_name = "Test sensor"
    value.sensor.hand_to_probe.position.x = x
    value.sensor.hand_to_probe.orientation.w = 1.0
    return value


def update_request(sensor_id="test", x=0.35):
    """Create one existing-sensor transform update request."""
    value = UpdateSensor.Request()
    value.sensor.sensor_id = sensor_id
    value.sensor.display_name = "Updated test sensor"
    value.sensor.hand_to_probe.position.x = x
    value.sensor.hand_to_probe.orientation.w = 1.0
    return value


def queued_command(command_controller):
    """Queue one physical command without configuring dispatch."""
    request = CommandRequest.create(
        command=SemanticCommand(command_id=CommandID.STAND_UP),
        client_id="test",
        origin=CommandOrigin.SYSTEM,
        recording_policy=RecordingPolicy.EXCLUDE,
    )
    command_controller.submit(request)


def test_updated_definition_is_used_by_later_attachment(tmp_path):
    registry, attachment, _, _ = controllers(tmp_path)
    registry.create(definition(x=0.20))
    registry.update(definition(x=0.35))

    pending = attachment.select_sensor("test")
    attachment.confirm_sensor("test", pending.attachment_revision)

    snapshot = attachment.require_motion_attachment()
    assert snapshot.hand_to_probe_position[0] == pytest.approx(0.35)


@pytest.mark.parametrize("stop_pending", [False, True])
def test_update_active_sensor_is_rejected_synchronously(tmp_path, stop_pending):
    registry, attachment, commands, repository = controllers(tmp_path)
    registry.create(definition())
    pending = attachment.select_sensor("test")
    attachment.confirm_sensor("test", pending.attachment_revision)
    if stop_pending:
        commands.cancel_all("startup_reconciliation")

    with pytest.raises(RuntimeError, match="currently selected"):
        registry.update(definition(x=0.35))

    assert repository.load("test").hand_to_probe.position.x == (
        pytest.approx(0.20)
    )


@pytest.mark.parametrize("stop_pending", [False, True])
def test_delete_pending_sensor_is_rejected_synchronously(tmp_path, stop_pending):
    registry, attachment, commands, repository = controllers(tmp_path)
    registry.create(definition())
    attachment.select_sensor("test")
    if stop_pending:
        commands.cancel_all("startup_reconciliation")

    with pytest.raises(RuntimeError, match="currently selected"):
        registry.delete("test")

    assert repository.exists("test") is True


def test_unrelated_idle_sensor_update_succeeds(tmp_path):
    registry, attachment, _, repository = controllers(tmp_path)
    registry.create(definition("active"))
    registry.create(definition("other"))
    pending = attachment.select_sensor("active")
    attachment.confirm_sensor("active", pending.attachment_revision)

    registry.update(definition("other", x=0.42))

    assert repository.load("other").hand_to_probe.position.x == (
        pytest.approx(0.42)
    )


@pytest.mark.parametrize("stop_pending", [False, True])
def test_sensor_update_is_blocked_during_attachment_reservation(tmp_path, stop_pending):
    registry, attachment, commands, repository = controllers(tmp_path)
    registry.create(definition("active"))
    registry.create(definition("other"))
    pending = attachment.select_sensor("active")
    attachment.confirm_sensor("active", pending.attachment_revision)

    with attachment.reserve_motion_attachment():
        if stop_pending:
            commands.cancel_all("startup_reconciliation")
        with pytest.raises(RuntimeError, match="workflow is active"):
            registry.update(definition("other", x=0.42))

    assert repository.load("other").hand_to_probe.position.x == (
        pytest.approx(0.20)
    )


@pytest.mark.parametrize("operation", ["create", "update", "delete"])
def test_queued_physical_command_blocks_registry_mutation(
    tmp_path,
    operation,
):
    registry, _, commands, repository = controllers(tmp_path)
    if operation != "create":
        repository.create(definition())
    queued_command(commands)

    with pytest.raises(RuntimeError, match="active or queued"):
        if operation == "create":
            registry.create(definition())
        elif operation == "update":
            registry.update(definition(x=0.35))
        else:
            registry.delete("test")


def mutate_definition(registry, operation):
    if operation == "create":
        return registry.create(definition(x=0.35))
    if operation == "update":
        return registry.update(definition(x=0.35))
    return registry.delete("test")


@pytest.mark.parametrize("operation", ["create", "update", "delete"])
@pytest.mark.parametrize("stop_phase", ["pending", "dispatched", "unconfirmed"])
def test_registry_mutation_during_idle_stop_preserves_motion_block(
    tmp_path, operation, stop_phase,
):
    registry, _, commands, repository = controllers(tmp_path)
    if operation != "create":
        repository.create(definition())
    dispatched = []
    commands.configure_dispatch(dispatched.append)
    stop_id = commands.cancel_all("startup_reconciliation")
    if stop_phase != "pending":
        commands.poll()
    if stop_phase == "unconfirmed":
        commands.handle_execution_status(CommandExecutionStatus(
            request_id=stop_id,
            state=CommandControllerState.STOP_UNCONFIRMED,
            detail="Prior shutdown requires reconciliation",
            execution_instance_id="owner", status_sequence=1,
        ))
    stop_before = commands.software_stop_status
    assert commands.queued_request_ids == ()

    mutate_definition(registry, operation)

    assert repository.exists("test") is (operation != "delete")
    if operation != "delete":
        assert repository.load("test").hand_to_probe.position.x == pytest.approx(0.35)
    assert commands.software_stop_status == stop_before
    assert commands.active_request_id == stop_id
    assert [item.command.command_id for item in dispatched] == [CommandID.EMERGENCY_CANCEL]
    with pytest.raises(SoftwareStopLatched):
        queued_command(commands)


@pytest.mark.parametrize("operation", ["create", "update", "delete"])
def test_registry_mutation_waits_for_unresolved_motion_with_empty_queue(tmp_path, operation):
    registry, _, commands, repository = controllers(tmp_path)
    if operation != "create":
        repository.create(definition())
    commands.configure_dispatch(lambda request: None)
    queued_command(commands)
    motion_id = commands.active_request_id
    commands.cancel_all()
    commands.poll()
    assert commands.queued_request_ids == ()
    assert commands.active_request_id == motion_id

    with pytest.raises(RuntimeError, match="active or queued"):
        mutate_definition(registry, operation)

    assert repository.exists("test") is (operation != "create")
    if operation != "create":
        assert repository.load("test").hand_to_probe.position.x == pytest.approx(0.20)


@pytest.mark.parametrize("operation", ["create", "update", "delete"])
def test_registry_mutation_waits_for_acquisition_abort_during_idle_stop(tmp_path, operation):
    registry, _, commands, repository = controllers(tmp_path)
    if operation != "create":
        repository.create(definition())
    stopped = [False]
    commands.configure_stop_participant(
        lambda request_id: None,
        lambda request_id: (stopped[0], "Acquisition abort pending"),
    )
    commands.cancel_all()
    commands.poll()
    assert commands.queued_request_ids == ()

    with pytest.raises(RuntimeError, match="Sensor acquisition stop is not confirmed"):
        mutate_definition(registry, operation)

    stopped[0] = True
    commands.poll()
    mutate_definition(registry, operation)
    assert repository.exists("test") is (operation != "delete")
    assert commands.software_stop_status.latched
    assert not commands.software_stop_status.stop_confirmed


def test_add_sensor_persists_and_publishes_definition(tmp_path):
    api, _, _, repository = api_state(tmp_path)

    response = SensorRegistryApi._handle_add_sensor(
        api,
        add_request(),
        AddSensor.Response(),
    )

    stored = repository.load("test")
    assert response.success is True
    assert "test_probe" in response.message
    assert stored.hand_to_probe.position.x == pytest.approx(0.20)
    assert api._static_broadcaster.transforms == [stored]
    assert api.published == [(stored,)]


@pytest.mark.parametrize("operation", ["add", "update", "delete"])
def test_default_hand_mutation_fails_without_publishing(tmp_path, operation):
    api, _, _, repository = api_state(tmp_path)
    original = replace(
        definition(DEFAULT_HAND_SENSOR_ID, x=0.05),
        display_name="hand_closed",
    )
    path = repository.get_sensor_path(DEFAULT_HAND_SENSOR_ID)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(original.to_dict()), encoding="utf-8")
    before = path.read_bytes()

    if operation == "add":
        response = SensorRegistryApi._handle_add_sensor(
            api, add_request(DEFAULT_HAND_SENSOR_ID), AddSensor.Response(),
        )
    elif operation == "update":
        response = SensorRegistryApi._handle_update_sensor(
            api, update_request(DEFAULT_HAND_SENSOR_ID), UpdateSensor.Response(),
        )
    else:
        request = DeleteSensor.Request()
        request.sensor_id = DEFAULT_HAND_SENSOR_ID
        response = SensorRegistryApi._handle_delete_sensor(
            api, request, DeleteSensor.Response(),
        )

    assert response.success is False
    assert "default hand configuration is read-only" in response.message
    assert path.read_bytes() == before
    assert api._static_broadcaster.transforms == []
    assert api.published == []


def test_registry_ros_definition_round_trips_channels():
    channel = SensorChannel(
        channel_id="magnetic_field",
        topic="/sensors/bmm150_probe/magnetic_field",
        message_type="sensor_msgs/msg/MagneticField",
    )
    original = sensor_definition_from_values(
        "bmm150_probe",
        "BMM150 probe",
        0.2,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        channels=(
            channel,
            SensorChannel(
                channel_id="spot_geometry",
                topic="",
                message_type="",
                source_kind=SensorChannelSource.SPOT_GEOMETRY,
            ),
        ),
    )

    message = SensorRegistryApi._definition_message(original)
    restored = SensorRegistryApi._definition_from_message(message)

    assert restored == original
    assert message.channels[0].channel_id == "magnetic_field"
    assert message.channels[0].source_kind == (
        SensorChannelSource.ROS_TOPIC.value
    )
    assert restored.channels[1].source_kind == (
        SensorChannelSource.SPOT_GEOMETRY
    )


def test_update_sensor_rejects_current_attachment(tmp_path):
    api, attachment, _, repository = api_state(tmp_path)
    api.controller.create(definition())
    pending = attachment.select_sensor("test")
    attachment.confirm_sensor("test", pending.attachment_revision)

    response = SensorRegistryApi._handle_update_sensor(
        api,
        update_request(),
        UpdateSensor.Response(),
    )

    assert response.success is False
    assert "currently selected" in response.message
    assert repository.load("test").hand_to_probe.position.x == (
        pytest.approx(0.20)
    )


def test_delete_sensor_removes_and_publishes_definition(tmp_path):
    api, _, _, repository = api_state(tmp_path)
    api.controller.create(definition())
    request = DeleteSensor.Request()
    request.sensor_id = "test"

    response = SensorRegistryApi._handle_delete_sensor(
        api,
        request,
        DeleteSensor.Response(),
    )

    assert response.success is True
    assert "Deleted sensor 'test'" in response.message
    assert repository.exists("test") is False
    assert api.published == [()]
