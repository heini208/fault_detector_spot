"""Tests for sensor registry transport in the UI ROS adapter."""

import math
import os
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PyQt5.QtWidgets import QApplication

from fault_detector_spot.ui.ros.sensor_attachment_client import (
    SensorAttachmentClient,
)


class FakeFuture:
    """Store callbacks without completing the request."""

    def __init__(self):
        self.callbacks = []

    def add_done_callback(self, callback):
        self.callbacks.append(callback)


class FakeClient:
    """Capture service requests."""

    def __init__(self, ready=True):
        self.ready = ready
        self.requests = []

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.requests.append(request)
        return FakeFuture()


class FakeNode:
    """Provide the ROS node surface used by the adapter."""

    def __init__(self):
        self.clients = {}
        self.subscriptions = []

    def create_subscription(self, message_type, topic, callback, qos):
        subscription = SimpleNamespace(
            message_type=message_type,
            topic=topic,
            callback=callback,
            qos=qos,
        )
        self.subscriptions.append(subscription)
        return subscription

    def create_client(self, service_type, name):
        client = FakeClient()
        self.clients[name] = client
        return client

    def destroy_subscription(self, subscription):
        self.subscriptions.remove(subscription)

    def destroy_client(self, client):
        return None


@pytest.fixture(scope="module", autouse=True)
def application():
    return QApplication.instance() or QApplication([])


def test_create_sensor_builds_hand_to_probe_request(application):
    node = FakeNode()
    client = SensorAttachmentClient(node)

    future = client.create_sensor(
        "bmm150_mount",
        "BMM150 Hall sensor",
        (0.20, -0.01, 0.03),
        (0.0, 0.0, 90.0),
    )

    assert future is not None
    request = node.clients[client.ADD_SENSOR_SERVICE].requests[0]
    assert request.sensor.sensor_id == "bmm150_mount"
    assert request.sensor.display_name == "BMM150 Hall sensor"
    assert request.sensor.hand_to_probe.position.x == pytest.approx(0.20)
    assert request.sensor.hand_to_probe.position.y == pytest.approx(-0.01)
    assert request.sensor.hand_to_probe.position.z == pytest.approx(0.03)
    assert request.sensor.hand_to_probe.orientation.x == pytest.approx(0.0)
    assert request.sensor.hand_to_probe.orientation.y == pytest.approx(0.0)
    assert request.sensor.hand_to_probe.orientation.z == pytest.approx(
        math.sqrt(0.5)
    )
    assert request.sensor.hand_to_probe.orientation.w == pytest.approx(
        math.sqrt(0.5)
    )


def test_create_sensor_rejects_incomplete_transform(application):
    client = SensorAttachmentClient(FakeNode())

    with pytest.raises(ValueError, match="exactly three"):
        client.create_sensor(
            "bmm150_mount",
            "BMM150 Hall sensor",
            (0.20, 0.0),
            (0.0, 0.0, 0.0),
        )


def test_unavailable_registry_reports_creation_failure(application):
    node = FakeNode()
    client = SensorAttachmentClient(node)
    node.clients[client.ADD_SENSOR_SERVICE].ready = False
    results = []
    client.creation_finished.connect(
        lambda success, message: results.append((success, message))
    )

    future = client.create_sensor(
        "bmm150_mount",
        "BMM150 Hall sensor",
        (0.20, 0.0, 0.0),
        (0.0, 0.0, 0.0),
    )

    assert future is None
    assert results == [
        (False, "Sensor registry service is unavailable"),
    ]
