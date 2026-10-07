"""The occupancy preference is available without mapping or robot transport."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from diagnostic_msgs.msg import DiagnosticStatus
from std_srvs.srv import SetBool

from fault_detector_spot.manipulation.arm_collision_control import (
    ArmCollisionControl,
)


def make_control():
    # No TF, map, action, or scene client capability is supplied to this owner.
    node = Mock(spec=[
        "create_publisher", "create_service", "create_timer",
        "destroy_publisher", "destroy_service", "destroy_timer",
    ])
    return ArmCollisionControl(node), node


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
