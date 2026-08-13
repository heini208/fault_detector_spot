"""Regression tests for refinement control delta dispatch."""

import math

from fault_detector_spot.ui.inspection.controls import InspectionControls


def test_refinement_delta_is_callable_as_bound_ui_helper():
    controls = object.__new__(InspectionControls)

    translation, pitch, yaw = controls._refinement_delta(
        "up",
        0.01,
        math.radians(2.0),
    )

    assert translation.x == 0.0
    assert translation.y == 0.0
    assert translation.z == 0.01
    assert pitch == 0.0
    assert yaw == 0.0


def test_refinement_rotation_delta_remains_unchanged():
    controls = object.__new__(InspectionControls)
    rotation_step = math.radians(2.0)

    translation, pitch, yaw = controls._refinement_delta(
        "yaw_left",
        0.01,
        rotation_step,
    )

    assert translation.x == 0.0
    assert translation.y == 0.0
    assert translation.z == 0.0
    assert pitch == 0.0
    assert yaw == rotation_step
