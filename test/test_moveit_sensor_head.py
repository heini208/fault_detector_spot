"""Exercise attachment and mesh read sequencing without ROS nodes or MoveIt."""

from copy import deepcopy
from dataclasses import replace
from types import SimpleNamespace

import pytest
from fault_detector_msgs.msg import SensorAttachmentState
from fault_detector_msgs.srv import GetSensorAttachment, GetSensorMesh
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters
from rclpy.serialization import deserialize_message, serialize_message

from fault_detector_spot.inspection.model.sensor_geometry import MAX_MESH_CHUNK_BYTES
from fault_detector_spot.manipulation.moveit_sensor_head import MoveItSensorHead
from fault_detector_spot.manipulation.sensor_head_geometry import SENSOR_HEAD_OBJECT_ID
from fault_detector_spot.shared.geometry.models import PoseData, QuaternionData, Vector3Data
from fault_detector_spot.shared.ros.sensor_geometry_adapter import head_geometry_to_messages

from test_sensor_geometry import binary_stl
from test_sensor_mesh_store import geometry_for


class Node:
    def __init__(self):
        self.clients = {}
        self.subscriptions = []
        self.destroyed = []

    def create_client(self, service_type, name):
        client = SimpleNamespace(service_type=service_type, name=name)
        self.clients[name] = client
        return client

    def create_subscription(self, message_type, topic, callback, qos):
        subscription = SimpleNamespace(message_type=message_type, callback=callback)
        self.subscriptions.append(subscription)
        return subscription

    def destroy_client(self, client):
        self.destroyed.append(client)

    def destroy_subscription(self, subscription):
        self.destroyed.append(subscription)


def wire(message):
    return deserialize_message(serialize_message(message), type(message))


def attachment_response(geometry, sensor_id="probe", revision=3):
    return GetSensorAttachment.Response(
        success=True,
        state=SensorAttachmentState(
            status=SensorAttachmentState.STATUS_ACTIVE,
            active_sensor_id=sensor_id, attachment_revision=revision,
        ),
        head_geometry=head_geometry_to_messages(geometry),
    )


@pytest.fixture
def robot_description():
    prefix = "package://spot_description/meshes/arm/collision/"

    def link(name, meshes=()):
        collisions = "".join(
            f'<collision><geometry><mesh filename="{prefix}{mesh}"/>'
            '</geometry></collision>' for mesh in meshes
        )
        return f'<link name="{name}">{collisions}</link>'

    return (
        '<robot name="spot">'
        + link("hand") + link("sensor_head_slot") + link("arm_link_wr0")
        + link("arm_link_wr1", ("arm_link_wr1.obj",))
        + link("arm_link_fngr", (
            "left_hinge.obj", "left_finger.obj", "left_tooth.obj",
            "right_hinge.obj", "right_finger.obj", "right_tooth.obj",
        ))
        + '<joint name="hand" type="fixed"><parent link="arm_link_wr1"/>'
        '<child link="hand"/><origin xyz=".19557 0 0"/></joint>'
        '<joint name="sensor_head_slot" type="fixed"><parent link="hand"/>'
        '<child link="sensor_head_slot"/></joint>'
        '<joint name="finger" type="revolute"><parent link="arm_link_wr1"/>'
        '<child link="arm_link_fngr"/><origin xyz=".11745 0 .01482"/></joint>'
        '</robot>'
    )


@pytest.fixture
def mount():
    triangles = []
    for index in range(300):
        x = index * .002
        points = ((x, 0, 0), (x + .001, 0, 0), (x, .001, 0), (x, 0, .001))
        triangles.extend(
            [points[index] for index in face]
            for face in ((0, 2, 1), (0, 1, 3), (0, 3, 2), (1, 2, 3))
        )
    data = binary_stl(triangles)
    geometry = replace(geometry_for(data), hand_to_geometry=PoseData(
        Vector3Data(-.112, .042, -.065), QuaternionData.identity(),
    ))
    return SimpleNamespace(data=data, geometry=geometry)


@pytest.fixture
def head():
    return MoveItSensorHead(Node())


def read_model(head, robot_description, response):
    phase, client, request = head.prepare()
    assert phase == "head_attachment"
    assert client.service_type is GetSensorAttachment
    wire(request)
    phase, client, request = head.advance(phase, wire(response))
    assert phase == "head_model"
    assert client.service_type is GetParameters
    assert wire(request).names == ["robot_description"]
    return head.advance(phase, wire(GetParameters.Response(values=[ParameterValue(
        type=ParameterType.PARAMETER_STRING, string_value=robot_description,
    )])))


def download(head, next_request, data):
    offsets = []
    while next_request is not None:
        phase, client, request = next_request
        assert phase == "head_mesh"
        assert client.service_type is GetSensorMesh
        request = wire(request)
        offsets.append(request.offset)
        assert request.byte_count == len(data)
        chunk = data[request.offset:request.offset + MAX_MESH_CHUNK_BYTES]
        response = GetSensorMesh.Response(
            success=True, complete=request.offset + len(chunk) == len(data), data=chunk,
        )
        next_request = head.advance(phase, wire(response))
    return offsets


def test_custom_head_reads_model_then_exact_bounded_asset(head, mount, robot_description):
    response = attachment_response(mount.geometry)
    head.node.subscriptions[0].callback(wire(response.state))

    request = read_model(head, robot_description, response)
    offsets = download(head, request, mount.data)

    assert offsets == [0, MAX_MESH_CHUNK_BYTES]
    assert head.expected.object.id == SENSOR_HEAD_OBJECT_ID
    assert len(head.expected.object.meshes) == 1
    assert head.expected.object.mesh_poses[0].position.x == -.112
    phase, client, request = head.verification_request()
    assert phase == "head_verify"
    assert client.service_type is GetSensorAttachment
    assert head.advance(phase, wire(response)) is None


def test_unknown_and_unconfirmed_attachments_do_not_request_idle_refresh(head):
    assert not head.needs_scene_sync

    for status in (
        SensorAttachmentState.STATUS_NO_SENSOR,
        SensorAttachmentState.STATUS_CONFIRMATION_PENDING,
    ):
        head.node.subscriptions[0].callback(SensorAttachmentState(status=status))
        assert not head.needs_scene_sync


def test_normal_preflight_verification_clears_idle_refresh_request(
    head, mount, robot_description,
):
    response = attachment_response(mount.geometry)
    observe = head.node.subscriptions[0].callback
    observe(wire(response.state))
    assert head.needs_scene_sync

    download(head, read_model(head, robot_description, response), mount.data)
    assert head.needs_scene_sync
    head.advance("head_verify", wire(response))

    assert not head.needs_scene_sync
    observe(wire(response.state))
    assert not head.needs_scene_sync


def test_bare_hand_reconfirmation_refreshes_even_when_revision_is_reused(
    head, robot_description,
):
    response = attachment_response(None, "", revision=0)
    observe = head.node.subscriptions[0].callback
    observe(wire(response.state))
    read_model(head, robot_description, response)
    head.advance("head_verify", wire(response))
    assert not head.needs_scene_sync

    observe(SensorAttachmentState(
        status=SensorAttachmentState.STATUS_NO_SENSOR, attachment_revision=0,
    ))
    assert not head.needs_scene_sync
    observe(wire(response.state))

    assert head.needs_scene_sync


def test_confirmed_bare_hand_restores_stock_jaw_without_mesh_download(
    head, robot_description,
):
    request = read_model(head, robot_description, attachment_response(None, ""))

    assert request is None
    assert len(head.expected.object.meshes) == 3
    assert head.expected.object.id == SENSOR_HEAD_OBJECT_ID


def test_cache_reuses_only_bytes_and_applies_new_placement(head, mount, robot_description):
    download(head, read_model(head, robot_description, attachment_response(mount.geometry)), mount.data)
    geometry = replace(mount.geometry, hand_to_geometry=PoseData(
        Vector3Data(.04, .02, .1), QuaternionData.identity(),
    ))

    request = read_model(head, robot_description, attachment_response(geometry, revision=4))

    assert request is None
    assert head.expected.object.mesh_poses[0].position.x == .04
    assert head.expected.object.mesh_poses[0].position.z == .1


def test_changed_digest_downloads_again(head, mount, robot_description):
    download(head, read_model(head, robot_description, attachment_response(mount.geometry)), mount.data)
    changed = b"x" + mount.data[1:]
    geometry = geometry_for(changed)

    request = read_model(head, robot_description, attachment_response(geometry, revision=4))

    assert request[0] == "head_mesh"
    assert request[2].sha256 == geometry.collision_sha256
    download(head, request, changed)
    assert head.expected is not None


def test_corrupt_mesh_fails_before_an_attachment_becomes_available(head, mount, robot_description):
    request = read_model(head, robot_description, attachment_response(mount.geometry))

    with pytest.raises(ValueError, match="checksum"):
        download(head, request, b"x" + mount.data[1:])

    assert head.expected is None


@pytest.mark.parametrize("description", ("<robot/>", "not XML"))
def test_unsupported_model_is_rejected_before_mesh_download(head, mount, description):
    with pytest.raises(ValueError, match="jaw slot"):
        read_model(head, description, attachment_response(mount.geometry))
    assert head.expected is None


def test_model_parameter_must_be_a_string(head, mount):
    head.prepare()
    head.advance("head_attachment", wire(attachment_response(mount.geometry)))

    with pytest.raises(RuntimeError, match="description is unavailable"):
        head.advance("head_model", GetParameters.Response(values=[]))


@pytest.mark.parametrize("change", ("pending", "id", "revision"))
def test_observed_attachment_change_blocks_remaining_reads(head, mount, robot_description, change):
    response = attachment_response(mount.geometry)
    read_model(head, robot_description, response)
    changed = deepcopy(response.state)
    if change == "pending":
        changed.status = SensorAttachmentState.STATUS_CONFIRMATION_PENDING
        changed.pending_sensor_id = "other"
    elif change == "id":
        changed.active_sensor_id = "other"
    else:
        changed.attachment_revision += 1
    head.node.subscriptions[0].callback(wire(changed))

    with pytest.raises(RuntimeError, match="attachment changed"):
        head.validate()


@pytest.mark.parametrize("change", ("geometry", "revision"))
def test_final_authoritative_read_rejects_changed_snapshot(head, mount, robot_description, change):
    response = attachment_response(mount.geometry)
    download(head, read_model(head, robot_description, response), mount.data)
    if change == "geometry":
        response.head_geometry[0].hand_to_geometry.position.x += .01
    else:
        response.state.attachment_revision += 1

    with pytest.raises(RuntimeError, match="changed during planning"):
        head.advance("head_verify", wire(response))


@pytest.mark.parametrize("response", (
    GetSensorMesh.Response(success=False, message="Mesh unavailable"),
    GetSensorMesh.Response(success=True, data=b"", complete=False),
    GetSensorMesh.Response(success=True, data=b"x", complete=True),
))
def test_download_rejects_failed_or_invalid_chunks(head, mount, robot_description, response):
    read_model(head, robot_description, attachment_response(mount.geometry))

    with pytest.raises(RuntimeError, match="unavailable|invalid chunk"):
        head.advance("head_mesh", wire(response))
    assert head.expected is None


@pytest.mark.parametrize("sensor_id,geometry_present", (("probe", False), ("", True)))
def test_attachment_identity_and_geometry_must_agree(head, mount, sensor_id, geometry_present):
    response = attachment_response(mount.geometry if geometry_present else None, sensor_id)
    head.prepare()

    with pytest.raises(RuntimeError, match="collision geometry"):
        head.advance("head_attachment", wire(response))


def test_failed_confirmation_read_is_not_a_bare_hand_fallback(head):
    head.prepare()

    with pytest.raises(RuntimeError, match="confirmation pending"):
        head.advance("head_attachment", GetSensorAttachment.Response(
            success=False, detail="confirmation pending",
        ))
    assert head.expected is None


def test_destroy_releases_all_read_clients_and_state_observer(head):
    head.destroy()

    assert len(head.node.destroyed) == 4
    assert all(client in head.node.destroyed for client in head.node.clients.values())
