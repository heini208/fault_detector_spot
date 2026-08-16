"""Tests for persistent sensor registry mutations."""

from types import SimpleNamespace

import pytest
from fault_detector_msgs.srv import AddSensor, DeleteSensor, UpdateSensor

from fault_detector_spot.inspection.repository.sensor_repository import (
    SensorRepository,
)
from fault_detector_spot.inspection.ros.sensor_registry_node import (
    SensorRegistryNode,
)


class FakeBroadcaster:
    """Capture static transforms sent by the registry."""

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


def registry_state(tmp_path):
    """Build the non-ROS state needed by registry callbacks."""
    state = SimpleNamespace()
    state.repository = SensorRepository(
        tmp_path / "sensors",
        tmp_path / "retired_sensors",
    )
    state._definitions = {}
    state._attachment_state = SimpleNamespace(
        active_sensor_id="",
        pending_sensor_id="",
    )
    state._static_broadcaster = FakeBroadcaster()
    state._definition_from_message = (
        SensorRegistryNode._definition_from_message
    )
    state._transform_message = lambda definition: definition
    state.published = 0

    def require_mutation_allowed(sensor_id):
        return SensorRegistryNode._require_mutation_allowed(
            state,
            sensor_id,
        )

    def delete_sensor(sensor_id):
        return SensorRegistryNode._delete_sensor(
            state,
            sensor_id,
        )

    def publish():
        state.published += 1

    def delete_sensor_response(request, response):
        return SensorRegistryNode._delete_sensor_response(
            state,
            request,
            response,
        )

    state._require_mutation_allowed = require_mutation_allowed
    state._delete_sensor = delete_sensor
    state._delete_sensor_response = delete_sensor_response
    state._publish_sensor_list = publish
    state.get_logger = lambda: FakeLogger()
    return state


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


def test_add_sensor_persists_definition_and_broadcasts_transform(tmp_path):
    state = registry_state(tmp_path)

    response = SensorRegistryNode._handle_add_sensor(
        state,
        add_request(),
        AddSensor.Response(),
    )

    stored = state.repository.load("test")
    assert response.success is True
    assert "test_probe" in response.message
    assert stored.hand_to_probe.position.x == pytest.approx(0.20)
    assert state._definitions["test"] == stored
    assert state._static_broadcaster.transforms == [stored]
    assert state.published == 1


def test_add_sensor_rejects_duplicate_mount_id(tmp_path):
    state = registry_state(tmp_path)
    SensorRegistryNode._handle_add_sensor(
        state,
        add_request(),
        AddSensor.Response(),
    )

    response = SensorRegistryNode._handle_add_sensor(
        state,
        add_request(x=0.40),
        AddSensor.Response(),
    )

    assert response.success is False
    assert "already exists" in response.message
    assert state.repository.load("test").hand_to_probe.position.x == (
        pytest.approx(0.20)
    )


def test_update_sensor_overwrites_transform_and_rebroadcasts(tmp_path):
    state = registry_state(tmp_path)
    SensorRegistryNode._handle_add_sensor(
        state,
        add_request(),
        AddSensor.Response(),
    )

    response = SensorRegistryNode._handle_update_sensor(
        state,
        update_request(),
        UpdateSensor.Response(),
    )

    stored = state.repository.load("test")
    assert response.success is True
    assert "Updated sensor 'test'" in response.message
    assert stored.display_name == "Updated test sensor"
    assert stored.hand_to_probe.position.x == pytest.approx(0.35)
    assert state._definitions["test"] == stored
    assert state._static_broadcaster.transforms[-1] == stored
    assert state.published == 2


def test_update_sensor_rejects_current_attachment(tmp_path):
    state = registry_state(tmp_path)
    SensorRegistryNode._handle_add_sensor(
        state,
        add_request(),
        AddSensor.Response(),
    )
    state._attachment_state.active_sensor_id = "test"

    response = SensorRegistryNode._handle_update_sensor(
        state,
        update_request(),
        UpdateSensor.Response(),
    )

    assert response.success is False
    assert "currently selected" in response.message
    assert state.repository.load("test").hand_to_probe.position.x == (
        pytest.approx(0.20)
    )


def test_delete_sensor_removes_definition_from_registry(tmp_path):
    state = registry_state(tmp_path)
    SensorRegistryNode._handle_add_sensor(
        state,
        add_request(),
        AddSensor.Response(),
    )
    request = DeleteSensor.Request()
    request.sensor_id = "test"

    response = SensorRegistryNode._handle_delete_sensor(
        state,
        request,
        DeleteSensor.Response(),
    )

    assert response.success is True
    assert "Deleted sensor 'test'" in response.message
    assert state.repository.exists("test") is False
    assert "test" not in state._definitions
    assert state.published == 2


def test_update_and_delete_fail_closed_without_attachment_state(tmp_path):
    state = registry_state(tmp_path)
    SensorRegistryNode._handle_add_sensor(
        state,
        add_request(),
        AddSensor.Response(),
    )
    state._attachment_state = None

    update = SensorRegistryNode._handle_update_sensor(
        state,
        update_request(),
        UpdateSensor.Response(),
    )
    delete_request = DeleteSensor.Request()
    delete_request.sensor_id = "test"
    delete = SensorRegistryNode._handle_delete_sensor(
        state,
        delete_request,
        DeleteSensor.Response(),
    )

    assert update.success is False
    assert delete.success is False
    assert "attachment state is unavailable" in update.message
    assert "attachment state is unavailable" in delete.message
    assert state.repository.exists("test") is True
