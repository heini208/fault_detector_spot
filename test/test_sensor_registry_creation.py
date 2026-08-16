"""Tests for application-owned sensor registry mutations."""

from threading import RLock

import pytest
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
)
from fault_detector_spot.application.controllers.sensor_attachment_controller import (
    SensorAttachmentController,
)
from fault_detector_spot.application.controllers.sensor_registry_controller import (
    SensorRegistryController,
)
from fault_detector_spot.inspection.model.sensor_models import (
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


def test_registry_and_attachment_share_one_repository(tmp_path):
    registry, attachment, _, repository = controllers(tmp_path)

    assert registry.sensor_repository is repository
    assert attachment.sensor_repository is repository


def test_updated_definition_is_used_by_later_attachment(tmp_path):
    registry, attachment, _, _ = controllers(tmp_path)
    registry.create(definition(x=0.20))
    registry.update(definition(x=0.35))

    pending = attachment.select_sensor("test")
    attachment.confirm_sensor("test", pending.attachment_revision)

    snapshot = attachment.require_motion_attachment()
    assert snapshot.hand_to_probe_position[0] == pytest.approx(0.35)


def test_update_active_sensor_is_rejected_synchronously(tmp_path):
    registry, attachment, _, repository = controllers(tmp_path)
    registry.create(definition())
    pending = attachment.select_sensor("test")
    attachment.confirm_sensor("test", pending.attachment_revision)

    with pytest.raises(RuntimeError, match="currently selected"):
        registry.update(definition(x=0.35))

    assert repository.load("test").hand_to_probe.position.x == (
        pytest.approx(0.20)
    )


def test_delete_pending_sensor_is_rejected_synchronously(tmp_path):
    registry, attachment, _, repository = controllers(tmp_path)
    registry.create(definition())
    attachment.select_sensor("test")

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


def test_sensor_update_is_blocked_during_attachment_reservation(tmp_path):
    registry, attachment, _, repository = controllers(tmp_path)
    registry.create(definition("active"))
    registry.create(definition("other"))
    pending = attachment.select_sensor("active")
    attachment.confirm_sensor("active", pending.attachment_revision)

    with attachment.reserve_motion_attachment():
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
