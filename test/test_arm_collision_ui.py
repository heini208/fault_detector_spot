"""The independent collision toggle reflects backend state without moving the robot."""

import os
from unittest.mock import Mock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from PyQt5.QtWidgets import QApplication, QLabel, QWidget
from rclpy.task import Future
from std_srvs.srv import SetBool
import pytest

from fault_detector_spot.ui.manipulation.controls import ManipulationControls
from fault_detector_spot.ui.ros.arm_collision_client import ArmCollisionClient


class Node:
    def __init__(self):
        self.subscriptions = {}
        self.requests = []
        self.available = True
        self.future = None
        self.destroyed = []

    def create_subscription(self, message_type, topic, callback, qos, **_kwargs):
        self.subscriptions[topic] = callback
        return topic

    def create_client(self, service_type, name):
        assert service_type is SetBool
        assert name == "fault_detector/set_arm_collision_checking"
        return self

    def service_is_ready(self):
        return self.available

    def call_async(self, request):
        self.requests.append(request)
        self.future = Future()
        return self.future

    def destroy_subscription(self, subscription):
        self.destroyed.append(subscription)

    def destroy_client(self, client):
        assert client is self
        self.destroyed.append("client")

    def state(self, available, enabled):
        self.subscriptions["fault_detector/arm_collision_checking"](
            DiagnosticStatus(values=[
                KeyValue(key="arm_collision_available", value=str(available).lower()),
                KeyValue(key="arm_collision_enabled", value=str(enabled).lower()),
            ])
        )


@pytest.fixture(scope="module")
def application():
    return QApplication.instance() or QApplication([])


@pytest.fixture
def controls(application):
    ui = QWidget()
    ui.node = Node()
    ui.status_label = QLabel()
    ui.visible_tags = {}
    ui.update_frames_dropdown = lambda box: box.addItem("body")
    ui.update_tags_dropdown = lambda box: None
    ui.execute_operation = Mock()
    ui.handle_simple_operation = Mock()
    control = ManipulationControls(ui)
    yield control
    control.destroy()
    control.posture_button.timer.stop()
    ui.close()


def test_control_starts_disabled_and_can_toggle_without_mapping(controls):
    button = controls.arm_collision_toggle
    assert not button.isEnabled()
    assert button.text() == "Waiting for collision status"
    assert "fault_detector/navigation_runtime" not in controls.node.subscriptions
    controls.node.state(True, False)
    assert button.text() == "Disabled"
    assert button.isEnabled() and not button.isChecked()

    button.click()
    assert controls.node.requests[-1].data is True
    controls.node.future.set_result(SetBool.Response(success=True, message="Enabled"))
    controls.node.state(True, True)
    assert button.text() == "Enabled"
    assert button.isEnabled() and button.isChecked()
    button.click()
    assert controls.node.requests[-1].data is False
    controls.node.future.set_result(SetBool.Response(success=True, message="Disabled"))
    controls.node.state(True, False)
    assert button.text() == "Disabled"
    assert button.isEnabled() and not button.isChecked()
    controls.ui.execute_operation.assert_not_called()
    controls.ui.handle_simple_operation.assert_not_called()


def test_toggle_waits_for_backend_state_without_optimistic_checkmark(controls):
    controls.node.state(True, True)
    button = controls.arm_collision_toggle
    button.click()

    assert controls.node.requests[-1].data is False
    assert button.text() == "Change pending…"
    assert button.isChecked() and not button.isEnabled()
    button.click()
    assert len(controls.node.requests) == 1
    controls.node.future.set_result(SetBool.Response(success=True, message="Disabled"))
    assert button.isChecked()  # Service acknowledgement does not invent telemetry.
    controls.node.state(True, False)
    assert button.text() == "Disabled"
    assert button.isEnabled() and not button.isChecked()


def test_rejection_preserves_setting_and_reports_reason(controls):
    controls.node.state(True, False)
    controls.arm_collision_toggle.click()
    assert controls.node.requests[-1].data is True

    controls.node.future.set_result(SetBool.Response(success=False, message="Control unavailable"))

    assert not controls.arm_collision_toggle.isChecked()
    assert controls.ui.status_label.text() == "Control unavailable"
    controls.ui.execute_operation.assert_not_called()


def test_stale_state_is_not_presented_as_enabled(controls):
    now = [0.0]
    controls.arm_collision_client._clock = lambda: now[0]
    controls.node.state(True, True)
    now[0] = 3.1

    controls.refresh_arm_collision_state()

    assert controls.arm_collision_toggle.text() == "Status unavailable"
    assert not controls.arm_collision_toggle.isEnabled()
    assert not controls.arm_collision_toggle.isChecked()
    controls.node.state(True, True)
    assert controls.arm_collision_toggle.isChecked()


def test_late_toggle_reply_cannot_reenable_an_unavailable_control(controls):
    controls.node.state(True, False)
    controls.arm_collision_toggle.click()
    controls.node.state(False, False)
    controls.node.future.set_result(SetBool.Response(success=True, message="Enabled"))

    assert controls.arm_collision_toggle.text() == "Control unavailable"
    assert not controls.arm_collision_toggle.isEnabled()
    assert not controls.arm_collision_toggle.isChecked()


def test_slow_reply_reports_unknown_outcome_while_status_can_recover(controls):
    now = [0.0]
    controls.arm_collision_client._clock = lambda: now[0]
    controls.node.state(True, False)
    controls.arm_collision_toggle.click()
    now[0] = 3.1
    controls.node.state(True, True)

    assert controls.arm_collision_toggle.text() == "Change outcome unknown"
    assert controls.arm_collision_toggle.isChecked()
    assert not controls.arm_collision_toggle.isEnabled()
    controls.arm_collision_toggle.click()
    assert len(controls.node.requests) == 1

    controls.node.future.set_result(SetBool.Response(success=True, message="Enabled"))
    assert controls.arm_collision_toggle.text() == "Enabled"
    assert controls.arm_collision_toggle.isEnabled()


def test_global_toggle_does_not_consume_one_shot_override(controls):
    controls.node.state(True, True)
    controls.ignore_environment_collisions_checkbox.setChecked(True)
    controls.arm_collision_toggle.click()

    assert controls.ignore_environment_collisions_checkbox.isChecked()
    controls.ui.execute_operation.assert_not_called()


def test_missing_service_reports_failure_without_changing_state(controls):
    controls.node.state(True, True)
    controls.node.available = False
    controls.arm_collision_toggle.click()

    assert controls.arm_collision_toggle.isChecked()
    assert "unavailable" in controls.ui.status_label.text()
    assert not controls.arm_collision_client.pending


def test_invalid_control_messages_do_not_refresh_freshness(application):
    node = Node()
    now = [0.0]
    client = ArmCollisionClient(node, monotonic_clock=lambda: now[0])
    node.state(True, True)
    now[0] = 4.0

    node.subscriptions["fault_detector/arm_collision_checking"](DiagnosticStatus())

    assert client.is_stale()
    client.destroy()


def test_destroy_releases_transport_and_ignores_late_results(application):
    node = Node()
    client = ArmCollisionClient(node)
    receiver = Mock()
    client.request_finished.connect(receiver)
    client.set_enabled(True)
    client.destroy()

    node.future.set_result(SetBool.Response(success=True, message="Enabled"))

    receiver.assert_not_called()
    assert node.destroyed == ["fault_detector/arm_collision_checking", "client"]


def test_controls_destroy_stops_refresh_and_releases_client(controls):
    timer = controls.arm_state_timer

    controls.destroy()

    assert not timer.isActive()
    assert controls.arm_collision_client is None
    assert "fault_detector/arm_collision_checking" in controls.node.destroyed
    assert "client" in controls.node.destroyed


def test_timeout_refresh_tolerates_reply_during_clock_read(application):
    node = Node()
    client = ArmCollisionClient(node, monotonic_clock=lambda: 0.0)
    client.set_enabled(True)

    def reply_during_refresh():
        node.future.set_result(SetBool.Response(success=True, message="Enabled"))
        return 4.0

    client._clock = reply_during_refresh
    assert client.pending_timed_out is True  # The observed request's timing snapshot.
    assert not client.pending
    assert client.pending_timed_out is False
    client.destroy()


def test_reply_cannot_clear_timestamp_of_request_admitted_during_release(application):
    class InterleavedClient(ArmCollisionClient):
        on_release = None

        def __setattr__(self, name, value):
            super().__setattr__(name, value)
            if name == "_pending" and value is None and self.on_release is not None:
                callback = self.on_release
                self.on_release = None
                callback()

    node = Node()
    now = [0.0]
    client = InterleavedClient(node, monotonic_clock=lambda: now[0])
    client.set_enabled(True)
    first_reply = node.future
    now[0] = 10.0
    client.on_release = lambda: client.set_enabled(False)

    first_reply.set_result(SetBool.Response(success=True, message="Enabled"))

    assert [request.data for request in node.requests] == [True, False]
    assert client.pending
    now[0] = 14.0
    assert client.pending_timed_out is True
    node.future.set_result(SetBool.Response(success=True, message="Disabled"))
    assert not client.pending
    client.destroy()
