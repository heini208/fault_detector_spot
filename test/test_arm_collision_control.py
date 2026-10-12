"""The occupancy preference is available without mapping or robot transport."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from diagnostic_msgs.msg import DiagnosticStatus
from fault_detector_msgs.msg import SensorAttachmentState
from std_srvs.srv import SetBool

from fault_detector_spot.manipulation.arm_collision_control import (
    ArmCollisionControl,
)


def make_control(confirmed=True):
    # No TF, map, action, or scene client capability is supplied to this owner.
    node = Mock(spec=[
        "create_publisher", "create_service", "create_timer", "create_subscription",
        "destroy_publisher", "destroy_service", "destroy_timer", "destroy_subscription",
    ])
    control = ArmCollisionControl(node)
    if confirmed:
        node.create_subscription.call_args.args[2](SensorAttachmentState(
            status=SensorAttachmentState.STATUS_ACTIVE, active_sensor_id="test_head",
        ))
    return control, node


def last_status(node):
    return node.create_publisher.return_value.publish.call_args.args[0]


def test_default_off_control_is_available_without_mapping():
    control, node = make_control()
    assert control.state().enabled is False
    assert control.state().revision == 0
    assert node.create_publisher.call_args.args[:2] == (
        DiagnosticStatus, "fault_detector/arm_collision_checking",
    )
    assert node.create_service.call_args.args[:2] == (
        SetBool, "fault_detector/set_arm_collision_checking",
    )
    assert node.create_timer.call_args.args[0] == 0.5
    status = last_status(node)
    assert status.name == "arm_collision_checking"
    assert status.level == DiagnosticStatus.OK
    assert {item.key: item.value for item in status.values} == {
        "arm_collision_available": "true",
        "arm_collision_enabled": "false",
        "arm_collision_can_enable": "true",
    }


def test_toggle_revisions_are_stable_until_changed_and_new_owner_resets():
    control, node = make_control()
    initial = control.state()
    enabled = control.set_enabled(True)
    assert enabled.enabled and enabled.revision == initial.revision + 1
    assert control.set_enabled(True) == enabled
    assert not initial.enabled
    assert last_status(node).message == "Enabled"
    disabled = control.set_enabled(False)
    assert not disabled.enabled and disabled.revision == enabled.revision + 1
    assert control.set_enabled(False) == disabled
    control.set_enabled(True)
    replacement, _ = make_control()
    assert replacement.state() == initial


@pytest.mark.parametrize("value", ["true", 1, 0, None])
def test_service_and_direct_setter_reject_non_boolean_policy(value):
    control, node = make_control()
    publisher = node.create_publisher.return_value.publish
    publisher.reset_mock()
    with pytest.raises(TypeError, match="boolean"):
        control.set_enabled(value)
    callback = node.create_service.call_args.args[2]
    response = callback(SimpleNamespace(data=value), SetBool.Response())
    assert not response.success and "boolean" in response.message
    assert control.state().revision == 0
    publisher.assert_not_called()


def test_service_toggle_publishes_immediately_and_teardown_is_idempotent():
    control, node = make_control()
    publisher = node.create_publisher.return_value.publish
    callback = node.create_service.call_args.args[2]
    publisher.reset_mock()
    response = callback(SetBool.Request(data=True), SetBool.Response())
    assert response.success and control.state().enabled
    publisher.assert_called_once()
    values = {item.key: item.value for item in last_status(node).values}
    assert values["arm_collision_available"] == "true"
    assert values["arm_collision_enabled"] == "true"
    control.destroy()
    control.destroy()
    node.destroy_subscription.assert_called_once_with(node.create_subscription.return_value)
    node.destroy_timer.assert_called_once_with(node.create_timer.return_value)
    node.destroy_service.assert_called_once_with(node.create_service.return_value)
    node.destroy_publisher.assert_called_once_with(node.create_publisher.return_value)
    with pytest.raises(RuntimeError, match="closed"):
        control.set_enabled(False)
    response = callback(SetBool.Request(data=False), SetBool.Response())
    assert not response.success and "closed" in response.message
    publisher.reset_mock()
    node.create_timer.call_args.args[1]()
    publisher.assert_not_called()


@pytest.mark.parametrize("state", [
    None,
    SensorAttachmentState(status=SensorAttachmentState.STATUS_NO_SENSOR),
    SensorAttachmentState(status=SensorAttachmentState.STATUS_CONFIRMATION_PENDING),
    SensorAttachmentState(status=SensorAttachmentState.STATUS_ACTIVE, pending_sensor_id="other"),
    SensorAttachmentState(status=99),
])
def test_enabling_requires_confirmed_attachment_for_direct_and_service_calls(state):
    control, node = make_control(confirmed=False)
    if state is not None:
        node.create_subscription.call_args.args[2](state)
    initial = control.state()
    with pytest.raises(RuntimeError, match="Confirm the head attachment"):
        control.set_enabled(True)
    response = node.create_service.call_args.args[2](
        SetBool.Request(data=True), SetBool.Response(),
    )
    assert not response.success and "Confirm the head attachment" in response.message
    assert control.state() == initial
    assert not control.set_enabled(False).enabled
    assert {item.key: item.value for item in last_status(node).values}["arm_collision_can_enable"] == "false"


@pytest.mark.parametrize("sensor_id", ["", "test_head"])
def test_confirmed_stock_or_custom_head_allows_enabling(sensor_id):
    control, node = make_control(confirmed=False)
    node.create_subscription.call_args.args[2](SensorAttachmentState(
        status=SensorAttachmentState.STATUS_ACTIVE, active_sensor_id=sensor_id,
    ))
    assert control.set_enabled(True).enabled


def test_pending_attachment_does_not_disable_collision_checks_but_blocks_reenabling():
    control, node = make_control()
    enabled = control.set_enabled(True)
    node.create_subscription.call_args.args[2](SensorAttachmentState(
        status=SensorAttachmentState.STATUS_CONFIRMATION_PENDING,
    ))
    assert control.state() == enabled
    assert not control.set_enabled(False).enabled
    with pytest.raises(RuntimeError, match="Confirm the head attachment"):
        control.set_enabled(True)
