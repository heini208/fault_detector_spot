"""Tests for independent sensor registry and attachment UI transports."""

from types import SimpleNamespace

from fault_detector_msgs.msg import SensorAttachmentState

from fault_detector_spot.ui.ros.sensor_attachment_client import (
    SensorAttachmentClient,
)
from fault_detector_spot.ui.ros.sensor_registry_client import (
    SensorRegistryClient,
)
from fault_detector_spot.ui.sensor.models import (
    SensorAttachmentViewStatus,
)


class FakeNode:
    def __init__(self):
        self.subscriptions = []
        self.clients = []
        self.destroyed_subscriptions = []
        self.destroyed_clients = []

    def create_subscription(
        self,
        message_type,
        topic,
        callback,
        qos,
    ):
        subscription = SimpleNamespace(
            message_type=message_type,
            topic=topic,
            callback=callback,
            qos=qos,
        )
        self.subscriptions.append(subscription)
        return subscription

    def create_client(self, service_type, service_name):
        client = SimpleNamespace(
            service_type=service_type,
            service_name=service_name,
        )
        self.clients.append(client)
        return client

    def destroy_subscription(self, subscription):
        self.destroyed_subscriptions.append(subscription)

    def destroy_client(self, client):
        self.destroyed_clients.append(client)


def test_registry_client_owns_only_definition_resources():
    node = FakeNode()
    client = SensorRegistryClient(node)

    assert [item.topic for item in node.subscriptions] == [
        SensorRegistryClient.SENSOR_TOPIC
    ]
    assert [item.service_name for item in node.clients] == [
        SensorRegistryClient.ADD_SENSOR_SERVICE,
        SensorRegistryClient.UPDATE_SENSOR_SERVICE,
        SensorRegistryClient.DELETE_SENSOR_SERVICE,
    ]

    client.destroy()

    assert node.destroyed_subscriptions == node.subscriptions
    assert node.destroyed_clients == node.clients


def test_attachment_client_owns_only_attachment_resources():
    node = FakeNode()
    client = SensorAttachmentClient(node)

    assert [item.topic for item in node.subscriptions] == [
        SensorAttachmentClient.STATE_TOPIC
    ]
    assert [item.service_name for item in node.clients] == [
        SensorAttachmentClient.SELECT_SERVICE,
        SensorAttachmentClient.CONFIRM_SERVICE,
    ]

    client.destroy()

    assert node.destroyed_subscriptions == node.subscriptions
    assert node.destroyed_clients == node.clients


def test_registry_client_maps_definition_without_attachment_state():
    node = FakeNode()
    client = SensorRegistryClient(node)
    orientation = SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
    position = SimpleNamespace(x=0.1, y=0.2, z=0.3)
    sensor = SimpleNamespace(
        sensor_id="hall_probe",
        display_name="Hall probe",
        hand_to_probe=SimpleNamespace(
            position=position,
            orientation=orientation,
        ),
    )

    view = client._definition_view(sensor)

    assert view.sensor_id == "hall_probe"
    assert view.display_name == "Hall probe"
    assert view.position == (0.1, 0.2, 0.3)
    assert view.rotation_degrees == (0.0, 0.0, 0.0)


def test_attachment_client_maps_authoritative_state():
    message = SimpleNamespace(
        status=SensorAttachmentState.STATUS_CONFIRMATION_PENDING,
        active_sensor_id="",
        pending_sensor_id=" hall_probe ",
        attachment_revision=4,
    )

    view = SensorAttachmentClient._state_view(message)

    assert view.status is SensorAttachmentViewStatus.PENDING
    assert view.active_sensor_id == ""
    assert view.pending_sensor_id == "hall_probe"
    assert view.attachment_revision == 4
