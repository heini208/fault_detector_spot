"""Tests for extracted probe geometry draft editing."""

from copy import deepcopy
from types import SimpleNamespace

import pytest

from fault_detector_spot.inspection.model.models import (
    ImagePoint,
    PoseData,
)
from fault_detector_spot.inspection.setup.probe_geometry_editor import (
    ProbeGeometryEditor,
)


class FakeObjectRepository:
    def load(self, object_id):
        assert object_id == "motor"
        return SimpleNamespace(
            get_routine=lambda routine_id: SimpleNamespace(
                routine_id=routine_id,
                sensor_id="hall_probe",
            )
        )


class FakeSensorRepository:
    def load(self, sensor_id):
        assert sensor_id == "hall_probe"
        return SimpleNamespace(
            hand_to_probe=PoseData.identity(),
        )


class FakeGeometry:
    def __init__(self):
        self.calls = []

    def resolve(self, **kwargs):
        self.calls.append(kwargs)
        return SimpleNamespace(
            probe_setup=SimpleNamespace(
                safe_approach_approved=False,
                surface_alignment_approved=False,
            )
        )


class Draft(SimpleNamespace):
    def clear_geometry(self):
        self.reference_pixel = None
        self.geometry = None
        self.setup = None
        self.refinement = None
        self.dirty = False
        self.validation_error = ""


def draft():
    return Draft(
        selected_object_id="motor",
        selected_routine_id="magnetic_scan",
        selected_reference_view_id="",
        reference_pixel=None,
        geometry=None,
        setup=None,
        refinement=None,
        dirty=False,
        validation_error="old",
    )


def editor():
    geometry = FakeGeometry()
    return (
        ProbeGeometryEditor(
            FakeObjectRepository(),
            FakeSensorRepository(),
            geometry,
        ),
        geometry,
    )


def test_select_reference_pixel_updates_draft_and_resolves_geometry():
    geometry_editor, geometry = editor()
    current = draft()
    pixel = ImagePoint(u=20, v=30)

    geometry_editor.select_reference_pixel(
        current,
        "slot1_hand",
        pixel,
        "surface_fit",
        0.10,
        0.20,
    )

    assert current.selected_reference_view_id == "slot1_hand"
    assert current.reference_pixel == pixel
    assert current.reference_pixel is not pixel
    assert current.dirty
    assert current.validation_error == ""
    assert geometry.calls[0]["object_id"] == "motor"
    assert geometry.calls[0]["routine_id"] == "magnetic_scan"
    assert geometry.calls[0]["reference_view_id"] == "slot1_hand"
    assert geometry.calls[0]["hand_to_probe_pose"] == PoseData.identity()


def test_clear_reference_pixel_delegates_to_draft_reset():
    geometry_editor, _ = editor()
    current = draft()
    current.selected_reference_view_id = "slot1_hand"
    current.reference_pixel = ImagePoint(u=20, v=30)
    current.geometry = object()
    current.setup = object()

    geometry_editor.clear_reference_pixel(current)

    assert current.selected_reference_view_id == ""
    assert current.reference_pixel is None
    assert current.geometry is None
    assert current.setup is None


def test_update_geometry_requires_reference_pixel():
    geometry_editor, _ = editor()

    with pytest.raises(ValueError, match="No reference pixel"):
        geometry_editor.update_geometry(
            draft(),
            "surface_fit",
            0.10,
            0.20,
        )
