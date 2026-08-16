"""Tests for active attachment ownership in probe geometry authoring."""

from types import SimpleNamespace

from fault_detector_spot.inspection.model.models import (
    ImagePoint,
    PoseData,
    Vector3Data,
)
from fault_detector_spot.inspection.setup.probe_geometry_editor import (
    ProbeGeometryEditor,
)


class _Geometry:
    def __init__(self):
        self.hand_to_probe_pose = None

    def resolve(self, **kwargs):
        self.hand_to_probe_pose = kwargs["hand_to_probe_pose"]
        return SimpleNamespace(probe_setup=SimpleNamespace())


class _AttachmentController:
    def __init__(self, hand_to_probe):
        self.hand_to_probe = hand_to_probe

    def require_motion_attachment(self):
        return SimpleNamespace(
            sensor_id="active_sensor",
            motion_sensor_id="active_sensor",
            hand_to_probe=lambda: self.hand_to_probe,
        )


def test_geometry_uses_confirmed_active_attachment():
    active_transform = PoseData.identity()
    active_transform.position = Vector3Data(x=0.27, y=-0.03, z=0.04)
    geometry = _Geometry()
    editor = ProbeGeometryEditor(
        object_repository=SimpleNamespace(),
        geometry=geometry,
        sensor_attachment_controller=_AttachmentController(
            active_transform
        ),
    )
    draft = SimpleNamespace(
        selected_object_id="object",
        selected_routine_id="routine_with_different_sensor",
    )

    editor._resolve_geometry(
        draft,
        "view",
        ImagePoint(u=10, v=20),
        "surface_fit",
        0.03,
        0.20,
    )

    assert geometry.hand_to_probe_pose == active_transform


def test_geometry_fails_closed_without_attachment_authority():
    editor = ProbeGeometryEditor(
        object_repository=SimpleNamespace(),
        geometry=_Geometry(),
    )
    draft = SimpleNamespace(
        selected_object_id="object",
        selected_routine_id="routine",
    )

    try:
        editor._resolve_geometry(
            draft,
            "view",
            ImagePoint(u=10, v=20),
            "surface_fit",
            0.03,
            0.20,
        )
    except RuntimeError as exception:
        assert "attachment state is unavailable" in str(exception)
    else:
        raise AssertionError("Missing attachment authority was accepted")
