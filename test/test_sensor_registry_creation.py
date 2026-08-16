"""Tests for persistent sensor creation through the registry service."""

from types import SimpleNamespace

import pytest
from fault_detector_msgs.srv import AddSensor

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
    """Build the non-ROS state needed by the add service callback."""
    state = SimpleNamespace()
    state.repository = SensorRepository(
        tmp_path / "sensors",
        tmp_path / "retired_sensors",
    )
    state._definitions = {}
    state._static_broadcaster = FakeBroadcaster()
    state._definition_from_message = (
        SensorRegistryNode._definition_from_message
    )
    state._transform_message = lambda definition: definition
    state.published = 0

    def publish():
        state.published += 1

    state._publish_sensor_list = publish
    state.get_logger = lambda: FakeLogger()
    return state


def request():
    """Create one manual hand-to-probe calibration request."""
    value = AddSensor.Request()
    value.sensor.sensor_id = "bmm150_mount"
    value.sensor.display_name = "BMM150 Hall sensor"
    value.sensor.hand_to_probe.position.x = 0.20
    value.sensor.hand_to_probe.position.y = -0.01
    value.sensor.hand_to_probe.position.z = 0.03
    value.sensor.hand_to_probe.orientation.z = 2 ** -0.5
    value.sensor.hand_to_probe.orientation.w = 2 ** -0.5
    return value


def test_add_sensor_persists_definition_and_broadcasts_transform(tmp_path):
    state = registry_state(tmp_path)

    response = SensorRegistryNode._handle_add_sensor(
        state,
        request(),
        AddSensor.Response(),
    )

    stored = state.repository.load("bmm150_mount")
    assert response.success is True
    assert "bmm150_mount_probe" in response.message
    assert stored.hand_to_probe.position.x == pytest.approx(0.20)
    assert stored.hand_to_probe.position.y == pytest.approx(-0.01)
    assert stored.hand_to_probe.position.z == pytest.approx(0.03)
    assert state._definitions["bmm150_mount"] == stored
    assert state._static_broadcaster.transforms == [stored]
    assert state.published == 1


def test_add_sensor_rejects_duplicate_mount_id(tmp_path):
    state = registry_state(tmp_path)
    first = request()
    SensorRegistryNode._handle_add_sensor(
        state,
        first,
        AddSensor.Response(),
    )

    response = SensorRegistryNode._handle_add_sensor(
        state,
        request(),
        AddSensor.Response(),
    )

    assert response.success is False
    assert "already exists" in response.message
    assert state.repository.list_sensor_ids() == ["bmm150_mount"]
