"""Exercise generated ROS messages across the UI and registry without nodes."""

from collections import deque
from concurrent.futures import Future
from dataclasses import replace
import os
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication
from fault_detector_msgs.srv import AddSensor, UpdateSensor, UploadSensorMesh
from rclpy.serialization import deserialize_message, serialize_message

from fault_detector_spot.inspection.model.sensor_geometry import MAX_MESH_CHUNK_BYTES
from fault_detector_spot.inspection.model.sensor_models import quaternion_from_rpy_degrees
from fault_detector_spot.inspection.repository.sensor_repository import SensorRepository
from fault_detector_spot.shared.geometry.models import PoseData, Vector3Data
from fault_detector_spot.shared.ros.sensor_geometry_adapter import (
    head_geometry_from_messages, head_geometry_to_messages,
)
from fault_detector_spot.ui.ros.sensor_registry_client import SensorRegistryClient

from test_sensor_geometry import binary_stl
from test_sensor_mesh_store import geometry_for
from test_sensor_registry_creation import api_state, definition, queued_command


@pytest.fixture(scope="module", autouse=True)
def application():
    return QApplication.instance() or QApplication([])


class OfflineService:
    def __init__(self, node, service_type, name):
        self.node = node
        self.service_type = service_type
        self.name = name
        self.ready = True

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        future = Future()
        # Exercise actual generated type support, rather than passing Python objects.
        request = deserialize_message(serialize_message(request), self.service_type.Request)
        self.node.sent.append((self.name, request))
        self.node.pending.append((self, request, future))
        return future


class OfflineNode:
    def __init__(self, api):
        self.sent = []
        self.pending = deque()
        self.handlers = {
            SensorRegistryClient.UPLOAD_MESH_SERVICE: api._handle_upload_mesh,
            SensorRegistryClient.ADD_SENSOR_SERVICE: api._handle_add_sensor,
            SensorRegistryClient.UPDATE_SENSOR_SERVICE: api._handle_update_sensor,
            SensorRegistryClient.DELETE_SENSOR_SERVICE: api._handle_delete_sensor,
        }

    def create_subscription(self, *args):
        return None

    def create_client(self, service_type, name):
        return OfflineService(self, service_type, name)

    def respond(self, exception=None):
        service, request, future = self.pending.popleft()
        if exception is not None:
            future.set_exception(exception)
            return
        response = self.handlers[service.name](request, service.service_type.Response())
        future.set_result(deserialize_message(
            serialize_message(response), service.service_type.Response,
        ))

    def drain(self):
        while self.pending:
            self.respond()


@pytest.fixture
def transport(tmp_path):
    api, attachment, commands, repository = api_state(tmp_path)
    node = OfflineNode(api)
    client = SensorRegistryClient(node)
    results = []
    client.creation_finished.connect(lambda *args: results.append(args))
    client.update_finished.connect(lambda *args: results.append(args))
    return SimpleNamespace(
        api=api, attachment=attachment, commands=commands,
        repository=repository, node=node, client=client, results=results,
    )


@pytest.fixture
def mount():
    triangles = []
    for offset in range(300):
        vertices = ((offset, 0, 0), (offset + .1, 0, 0),
                    (offset, .1, 0), (offset, 0, .1))
        triangles.extend(
            [vertices[index] for index in face]
            for face in ((0, 2, 1), (0, 1, 3), (0, 3, 2), (1, 2, 3))
        )
    mesh = binary_stl(triangles)
    tip_rotation = quaternion_from_rpy_degrees(11.25, -7.1, 45.7)
    geometry = replace(geometry_for(mesh), hand_to_geometry=PoseData(
        Vector3Data(-.11208867798506154, .041992, -.06489429454966204),
        quaternion_from_rpy_degrees(90.0, 0.0, 0.0),
    ))
    return dict(
        sensor_id="probe", display_name="Test probe",
        translation_m=(.046238241757377466, -.00107, -.07917394514872324),
        rotation_degrees=(0.0, 0.0, 0.0),
        head_geometry=geometry, collision_mesh=mesh,
        orientation=(tip_rotation.x, tip_rotation.y, tip_rotation.z, tip_rotation.w),
        channels=(SimpleNamespace(
            channel_id="spot_geometry", topic="", message_type="",
            source_kind="spot_geometry",
        ),),
    )


def test_upload_then_atomic_save_and_metadata_reload(transport, mount):
    progress = []
    transport.client.upload_progress.connect(lambda *args: progress.append(args))
    future = transport.client.create_sensor(**mount)
    node = transport.node
    repository = transport.repository

    while node.pending[0][0].name == SensorRegistryClient.UPLOAD_MESH_SERVICE:
        assert not repository.exists("probe")
        node.respond()
    assert (repository.root_dir / mount["head_geometry"].asset_path).is_file()
    assert not repository.exists("probe")
    node.respond()

    assert future.result().success
    assert len(transport.results) == 1 and transport.results[0][0]
    stored = SensorRepository(repository.root_dir).load("probe")
    assert stored.head_geometry == mount["head_geometry"]
    assert stored.hand_to_probe.position.x == mount["translation_m"][0]
    assert tuple(stored.hand_to_probe.orientation.to_dict().values()) == mount["orientation"]
    assert stored.channels[0].channel_id == "spot_geometry"
    assert repository.read_collision_mesh(stored.head_geometry) == mount["collision_mesh"]
    view = transport.client._definition_view(transport.api._definition_message(stored))
    assert view.head_geometry == mount["head_geometry"]
    assert progress[-1] == (len(mount["collision_mesh"]),) * 2
    uploads = [request for name, request in node.sent if name.endswith("upload_sensor_mesh")]
    assert len(uploads) == 2
    assert all(len(request.data) <= MAX_MESH_CHUNK_BYTES for request in uploads)
    assert uploads[1].offset == len(uploads[0].data)
    assert len(serialize_message(node.sent[-1][1])) < 4096


def test_edit_saved_geometry_uses_asset_reference_without_upload(transport, mount):
    transport.client.create_sensor(**mount)
    transport.node.drain()
    transport.node.sent.clear()
    mount["collision_mesh"] = b""
    mount["translation_m"] = (.15, .2, .3)

    transport.client.update_sensor(**mount)
    transport.node.drain()

    assert [name for name, _ in transport.node.sent] == [
        SensorRegistryClient.UPDATE_SENSOR_SERVICE,
    ]
    stored = transport.repository.load("probe")
    assert stored.head_geometry == mount["head_geometry"]
    assert stored.hand_to_probe.position.x == .15


@pytest.mark.parametrize("failure", ("exception", "checksum", "timeout"))
def test_failed_upload_keeps_previous_definition(transport, mount, failure):
    original = definition("probe", x=.25)
    transport.repository.create(original)
    future = transport.client.update_sensor(**mount)
    if failure == "exception":
        transport.node.respond(RuntimeError("Transfer failed"))
    elif failure == "checksum":
        # Damage the wire payload after client-side validation.
        transport.node.pending[0][1].data[-1] ^= 1
        transport.node.drain()
    else:
        transport.client.findChild(QTimer).timeout.emit()
        transport.node.drain()  # A late chunk acknowledgement must not save.

    with pytest.raises((RuntimeError, TimeoutError)):
        future.result()
    assert len(transport.results) == 1 and not transport.results[0][0]
    assert transport.repository.load("probe") == original
    assert all(name != SensorRegistryClient.UPDATE_SENSOR_SERVICE
               for name, _ in transport.node.sent)


def test_missing_asset_is_rejected_by_backend_without_changing_definition(transport, mount):
    original = definition("probe")
    transport.repository.create(original)
    mount["collision_mesh"] = b""

    future = transport.client.update_sensor(**mount)
    transport.node.drain()

    assert not future.result().success
    assert transport.repository.load("probe") == original


@pytest.mark.parametrize("guard", ("selected", "queued"))
def test_geometry_upload_does_not_bypass_definition_guards(transport, mount, guard):
    original = definition("probe")
    transport.repository.create(original)
    if guard == "selected":
        transport.attachment.select_sensor("probe")
    else:
        queued_command(transport.commands)

    future = transport.client.update_sensor(**mount)
    transport.node.drain()

    assert not future.result().success
    assert transport.repository.load("probe") == original


def test_upload_service_unavailable_reports_failure(transport, mount):
    transport.client._upload_mesh_client.ready = False

    assert transport.client.create_sensor(**mount) is None
    assert transport.results == [(False, "Collision mesh upload service is unavailable")]
    assert not transport.node.sent


def test_synchronous_transport_failure_reports_save_result(transport, monkeypatch):
    def disconnected(request):
        raise RuntimeError("Registry client disconnected")

    monkeypatch.setattr(transport.client._add_sensor_client, "call_async", disconnected)

    assert transport.client.create_sensor(
        "probe", "Probe", (0, 0, 0), (0, 0, 0),
    ) is None
    assert transport.results == [(False, "Registry client disconnected")]
    assert not transport.repository.exists("probe")


def test_optional_geometry_adapter_and_transport_bound(mount):
    assert head_geometry_from_messages(head_geometry_to_messages(None)) is None
    geometry = mount["head_geometry"]
    assert head_geometry_from_messages(head_geometry_to_messages(geometry)) == geometry
    assert "49152" in UploadSensorMesh.Request.get_fields_and_field_types()["data"]
