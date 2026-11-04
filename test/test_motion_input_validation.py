"""Manual movement rejects invalid numbers before confirmation or dispatch."""

import math
import os
from types import SimpleNamespace
from unittest.mock import Mock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PyQt5.QtWidgets import QApplication, QLabel
from fault_detector_msgs.msg import OperationalIntent, TagElement

from fault_detector_spot.ui.manipulation.controls import ManipulationControls
from fault_detector_spot.ui.navigation.base_movement_controls import BaseMovementControls


INVALID_NUMBERS = ("invalid", "", "  ", "nan", "inf", "-inf", "1e309")
MOVEMENT_FIELDS = [
    ("arm", axis) for axis in ("X", "Y", "Z", "Roll", "Pitch", "Yaw")
] + [("base", axis) for axis in ("X", "Y", "Yaw")]
MOVEMENT_HANDLERS = [
    ("arm", "handle_full_message", (OperationalIntent.INTENT_MOVE_ARM_RELATIVE,)),
    ("arm", "handle_tag_selection", ()),
    ("arm", "handle_move_and_wait", ()),
    ("base", "handle_move_base_relative", ()),
    ("base", "handle_move_to_tag", ()),
]


@pytest.fixture(scope="module")
def application():
    return QApplication.instance() or QApplication([])


@pytest.fixture
def make_controls(application):
    created = []

    def make(kind):
        tag = TagElement()
        tag.id = 1
        tag.pose.header.frame_id = "body"
        tag.pose.pose.orientation.w = 1.0
        ui = SimpleNamespace(
            node=None, status_label=QLabel(), visible_tags={1: tag},
            update_frames_dropdown=lambda box: box.addItem("body"),
            update_tags_dropdown=lambda box: box.addItem("1"),
            execute_operation=Mock(return_value="request"),
            handle_simple_operation=Mock(),
        )
        cls = ManipulationControls if kind == "arm" else BaseMovementControls
        controls = cls(ui)
        controls.ask_question = Mock(return_value=True)
        controls.show_warning = Mock()
        created.append(controls)
        return controls

    yield make
    for controls in created:
        controls.posture_button.timer.stop()
        if isinstance(controls, ManipulationControls):
            controls.destroy()


def assert_rejected(controls, axis, original_text):
    controls.show_warning.assert_called_once()
    assert axis in controls.show_warning.call_args.args[1]
    controls.ask_question.assert_not_called()
    controls.ui.execute_operation.assert_not_called()
    assert controls.offset_fields[axis].text() == original_text


@pytest.mark.parametrize("kind,axis", MOVEMENT_FIELDS)
@pytest.mark.parametrize("text", INVALID_NUMBERS)
def test_each_movement_field_rejects_invalid_numbers(make_controls, kind, axis, text):
    controls = make_controls(kind)
    controls.offset_fields[axis].setText(text)

    if kind == "arm":
        controls.handle_full_message(OperationalIntent.INTENT_MOVE_ARM_RELATIVE)
    else:
        controls.handle_move_base_relative()

    assert_rejected(controls, axis, text)


@pytest.mark.parametrize("kind,handler,args", MOVEMENT_HANDLERS)
def test_every_movement_handler_stops_before_confirmation(make_controls, kind, handler, args):
    controls = make_controls(kind)
    controls.offset_fields["Yaw"].setText("invalid")

    getattr(controls, handler)(*args)

    assert_rejected(controls, "Yaw", "invalid")


@pytest.mark.parametrize("kind,axis", MOVEMENT_FIELDS)
def test_adjustment_preserves_invalid_field(make_controls, kind, axis):
    controls = make_controls(kind)
    controls.offset_fields[axis].setText("invalid")
    adjustment = controls._change_offset if axis in ("X", "Y", "Z") else controls._change_angle

    adjustment(axis, -0.05 if axis in ("X", "Y", "Z") else -5.0)

    assert_rejected(controls, axis, "invalid")


@pytest.mark.parametrize("kind,axis", [("arm", "X"), ("arm", "Roll"), ("base", "X"), ("base", "Yaw")])
@pytest.mark.parametrize("text", INVALID_NUMBERS[1:])
def test_adjustment_rejects_blank_and_nonfinite_numbers(make_controls, kind, axis, text):
    controls = make_controls(kind)
    controls.offset_fields[axis].setText(text)
    adjustment = controls._change_offset if axis == "X" else controls._change_angle

    adjustment(axis, 0.05 if axis == "X" else 5.0)

    assert_rejected(controls, axis, text)


@pytest.mark.parametrize("kind,axis,text,delta,expected", [
    ("arm", "X", "0", -0.05, "-0.05"),
    ("arm", "Y", "-0.10", 0.05, "-0.05"),
    ("base", "X", "0", -0.10, "-0.10"),
    ("base", "Y", "-0.20", 0.10, "-0.10"),
    ("arm", "Roll", "179", 5.0, "-176.0"),
    ("arm", "Pitch", "-179", -5.0, "176.0"),
    ("base", "Yaw", "179", 5.0, "-176.0"),
    ("base", "Yaw", "-179", -5.0, "176.0"),
])
def test_valid_adjustment_keeps_zero_negative_and_angle_wrapping(make_controls, kind, axis, text, delta, expected):
    controls = make_controls(kind)
    controls.offset_fields[axis].setText(text)
    adjustment = controls._change_offset if axis in ("X", "Y", "Z") else controls._change_angle

    adjustment(axis, delta)

    assert controls.offset_fields[axis].text() == expected
    controls.show_warning.assert_not_called()
    controls.ui.execute_operation.assert_not_called()


@pytest.mark.parametrize("kind,handler,args", MOVEMENT_HANDLERS)
def test_valid_movement_preserves_numbers_and_confirmation(make_controls, kind, handler, args):
    controls = make_controls(kind)
    values = {"X": " -0.25 ", "Y": "0", "Yaw": "-90"}
    if kind == "arm":
        values.update({"Z": "0.75", "Roll": "90", "Pitch": "0"})
    for axis, value in values.items():
        controls.offset_fields[axis].setText(value)

    getattr(controls, handler)(*args)

    controls.ui.execute_operation.assert_called_once()
    intent = controls.ui.execute_operation.call_args.args[0]
    assert intent.offset.header.frame_id == "body"
    assert intent.offset.pose.position.x == -0.25
    assert intent.offset.pose.position.y == 0.0
    assert intent.offset.pose.position.z == (0.75 if kind == "arm" else 0.0)
    quaternion = intent.offset.pose.orientation
    expected = (0.5, -0.5, -0.5, 0.5) if kind == "arm" else (0.0, 0.0, -math.sqrt(0.5), math.sqrt(0.5))
    assert (quaternion.x, quaternion.y, quaternion.z, quaternion.w) == pytest.approx(expected)
    assert {axis: field.text() for axis, field in controls.offset_fields.items()} == values
    controls.show_warning.assert_not_called()
    if handler == "handle_full_message":
        controls.ask_question.assert_not_called()
    else:
        controls.ask_question.assert_called_once()


@pytest.mark.parametrize("kind", ["arm", "base"])
def test_invalid_builder_returns_no_intent(make_controls, kind):
    controls = make_controls(kind)
    controls.offset_fields["X"].setText("invalid")

    if kind == "arm":
        intent = controls.build_move_to_tag_intent()
    else:
        intent = controls.build_move_base_intent(OperationalIntent.INTENT_MOVE_BASE_RELATIVE)

    assert intent is None
    assert_rejected(controls, "X", "invalid")
