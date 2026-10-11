"""Read coherent confirmed collision snapshots without ROS execution."""

from dataclasses import replace
from types import SimpleNamespace

import pytest
import yaml
from fault_detector_msgs.msg import SensorAttachmentState
from fault_detector_msgs.srv import GetSensorAttachment
from rclpy.serialization import deserialize_message, serialize_message

from fault_detector_spot.application.api.sensor_attachment_api import SensorAttachmentApi
from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.commanding.command_request import (
    CommandOrigin, CommandRequest, RecordingPolicy,
)
from fault_detector_spot.application.commanding.semantic_command import SemanticCommand
from fault_detector_spot.inspection.model.sensor_models import (
    DEFAULT_HAND_SENSOR_ID,
    MotionAttachmentSnapshot,
    sensor_definition_from_values,
)
from fault_detector_spot.shared.ros.sensor_geometry_adapter import head_geometry_from_messages

from test_sensor_attachment_controller import build_controller, sensor_definition
from test_sensor_mesh_store import geometry_for, mesh


@pytest.fixture
def registered(tmp_path, mesh):
    controller, repository, _, commands = build_controller(tmp_path)
    geometry = geometry_for(mesh)
    geometry.hand_to_geometry.position.x = -0.112
    repository.store_collision_chunk(
        geometry.collision_sha256, len(mesh), 0, mesh,
    )
    definition = replace(sensor_definition(), head_geometry=geometry)
    repository.create(definition)
    api = SensorAttachmentApi.__new__(SensorAttachmentApi)
    api.controller = controller
    return SimpleNamespace(
        api=api, controller=controller, repository=repository,
        definition=definition, commands=commands,
    )


def confirm(registered):
    pending = registered.controller.select_sensor(registered.definition.sensor_id)
    return registered.controller.confirm_sensor(
        registered.definition.sensor_id, pending.attachment_revision,
    )


def read(api):
    request = deserialize_message(
        serialize_message(GetSensorAttachment.Request()), GetSensorAttachment.Request,
    )
    response = api._read(request, GetSensorAttachment.Response())
    return deserialize_message(serialize_message(response), GetSensorAttachment.Response)


def test_read_returns_exact_confirmed_revision_and_geometry(registered):
    confirmed = confirm(registered)

    response = read(registered.api)

    assert response.success, response.detail
    assert response.state.status == SensorAttachmentState.STATUS_ACTIVE
    assert response.state.active_sensor_id == registered.definition.sensor_id
    assert response.state.attachment_revision == confirmed.attachment_revision
    assert head_geometry_from_messages(response.head_geometry) == (
        registered.definition.head_geometry
    )
    assert registered.controller.snapshot() == confirmed


def test_success_uses_the_same_snapshot_even_if_state_changes_after_read(registered):
    confirmed = confirm(registered)
    original = registered.controller.planning_attachment

    def read_then_select_bare_hand():
        result = original()
        registered.controller.clear_sensor()
        return result

    registered.controller.planning_attachment = read_then_select_bare_hand

    response = read(registered.api)

    assert response.success
    assert response.state.active_sensor_id == registered.definition.sensor_id
    assert response.state.attachment_revision == confirmed.attachment_revision
    assert response.head_geometry
    assert registered.controller.snapshot().attachment_revision > confirmed.attachment_revision


def test_read_is_available_while_a_command_holds_the_attachment_fixed(registered):
    confirm(registered)
    registered.commands.submit(CommandRequest.create(
        command=SemanticCommand(command_id=CommandID.WAIT_TIME, wait_time=1.0),
        client_id="test", origin=CommandOrigin.SYSTEM,
        recording_policy=RecordingPolicy.EXCLUDE,
    ))

    response = read(registered.api)

    assert response.success
    assert response.head_geometry
    with pytest.raises(RuntimeError, match="active or queued"):
        registered.controller.clear_sensor()


@pytest.mark.parametrize("select_sensor", (False, True))
def test_unconfirmed_state_never_exposes_collision_geometry(registered, select_sensor):
    if select_sensor:
        registered.controller.select_sensor(registered.definition.sensor_id)

    response = read(registered.api)

    assert not response.success
    assert "confirmation is pending" in response.detail
    assert not response.head_geometry


def test_switch_does_not_return_geometry_from_the_previous_confirmed_mount(registered):
    confirm(registered)
    registered.repository.create(sensor_definition("other"))
    pending = registered.controller.select_sensor("other")

    response = read(registered.api)

    assert not response.success
    assert response.state.status == SensorAttachmentState.STATUS_CONFIRMATION_PENDING
    assert response.state.pending_sensor_id == "other"
    assert response.state.attachment_revision == pending.attachment_revision
    assert not response.head_geometry


def test_confirmed_bare_hand_returns_no_geometry(registered):
    confirm(registered)
    pending = registered.controller.clear_sensor()
    confirmed = registered.controller.confirm_sensor("", pending.attachment_revision)

    response = read(registered.api)

    assert response.success
    assert response.state.status == SensorAttachmentState.STATUS_ACTIVE
    assert response.state.active_sensor_id == ""
    assert response.state.attachment_revision == confirmed.attachment_revision
    assert not response.head_geometry


def test_confirmed_default_hand_keeps_probe_pose_and_uses_stock_geometry(registered):
    definition = sensor_definition_from_values(
        DEFAULT_HAND_SENSOR_ID, "hand_closed", .05, 0, 0, 0, 0, 0,
    )
    path = registered.repository.get_sensor_path(DEFAULT_HAND_SENSOR_ID)
    path.write_text(yaml.safe_dump(definition.to_dict()), encoding="utf-8")
    original = path.read_bytes()
    pending = registered.controller.select_sensor(DEFAULT_HAND_SENSOR_ID)
    confirmed = registered.controller.confirm_sensor(
        DEFAULT_HAND_SENSOR_ID, pending.attachment_revision,
    )

    response = read(registered.api)
    attachment = registered.controller.require_motion_attachment()

    assert response.success, response.detail
    assert response.state.status == SensorAttachmentState.STATUS_ACTIVE
    assert response.state.active_sensor_id == DEFAULT_HAND_SENSOR_ID
    assert response.state.attachment_revision == confirmed.attachment_revision
    assert not response.head_geometry
    assert attachment.sensor_id == DEFAULT_HAND_SENSOR_ID
    assert attachment.probe_frame == "hand_tip_probe"
    assert attachment.hand_to_probe_position == (.05, 0, 0)
    assert attachment.hand_to_probe_orientation == (0, 0, 0, 1)
    assert path.read_bytes() == original


def test_planning_rejects_a_legacy_mount_without_breaking_tip_only_consumers(registered):
    registered.repository.create(sensor_definition("legacy"))
    pending = registered.controller.select_sensor("legacy")
    registered.controller.confirm_sensor("legacy", pending.attachment_revision)

    assert registered.controller.require_motion_attachment().sensor_id == "legacy"
    response = read(registered.api)

    assert not response.success
    assert "has no collision geometry" in response.detail
    assert "Import and save a sensorhead file" in response.detail
    assert not response.head_geometry


def test_missing_stored_asset_fails_the_read(registered):
    confirm(registered)
    asset = registered.repository.root_dir / registered.definition.head_geometry.asset_path
    asset.unlink()

    response = read(registered.api)

    assert not response.success
    assert not response.head_geometry


def test_snapshot_owns_its_collision_pose(registered):
    snapshot = MotionAttachmentSnapshot.from_definition(registered.definition, 3)

    snapshot.head_geometry.hand_to_geometry.position.x = 9.0

    assert registered.definition.head_geometry.hand_to_geometry.position.x == -0.112


def test_attachment_api_owns_and_releases_read_service(registered):
    services = {}
    removed = []
    publisher = SimpleNamespace(publish=lambda message: None)

    def create_service(service_type, name, callback, **kwargs):
        services[name] = SimpleNamespace(service_type=service_type, callback=callback)
        return name

    node = SimpleNamespace(
        create_service=create_service,
        create_publisher=lambda *args: publisher,
        destroy_service=removed.append,
        destroy_publisher=lambda value: None,
    )
    api = SensorAttachmentApi(node, registered.controller)

    assert services[api.GET_SERVICE].service_type is GetSensorAttachment
    response = services[api.GET_SERVICE].callback(
        GetSensorAttachment.Request(), GetSensorAttachment.Response(),
    )
    assert not response.success
    api.close()
    assert api.GET_SERVICE in removed
