"""Tests for active attachment ownership in probe geometry authoring."""

from dataclasses import dataclass
from types import SimpleNamespace

from fault_detector_spot.inspection.model.models import (
    ImagePoint,
    InspectionRoutine,
    ReferenceTag,
)
from fault_detector_spot.shared.geometry.models import (
    PoseData,
    Vector3Data,
)
from fault_detector_spot.inspection.setup.probe_geometry_editor import (
    ProbeGeometryEditor,
)
from fault_detector_spot.inspection.setup.reference_probe_setup import (
    ReferenceProbeSetup,
    initialize_reference_probe_setup,
)
from fault_detector_spot.inspection.setup.reference_view_surface_target import (
    ReferenceSurfaceTarget,
)


@dataclass(frozen=True)
class _GeometryResult:
    probe_setup: ReferenceProbeSetup


class _Geometry:
    def __init__(self):
        self.hand_to_probe_pose = None

    def resolve(self, **kwargs):
        self.hand_to_probe_pose = kwargs["hand_to_probe_pose"]
        target_distance = kwargs["target_surface_distance_m"]
        aligned_distance = kwargs["aligned_preapproach_distance_m"]
        target_pose = PoseData.identity()
        target_pose.position.x = 1.0 - target_distance
        aligned_pose = PoseData.identity()
        aligned_pose.position.x = 1.0 - aligned_distance
        surface_target = ReferenceSurfaceTarget(
            surface_point_object=Vector3Data(x=1.0, y=0.0, z=0.0),
            outward_direction_object=Vector3Data(x=-1.0, y=0.0, z=0.0),
            target_pose_object=target_pose,
            aligned_preapproach_pose_object=aligned_pose,
            target_surface_distance_m=target_distance,
            aligned_preapproach_distance_m=aligned_distance,
            direction_source=kwargs["approach_mode"],
        )
        return _GeometryResult(initialize_reference_probe_setup(surface_target))


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
    routine = InspectionRoutine(
        routine_id="routine_with_different_sensor",
        display_name="Inspection",
        reference_tag=ReferenceTag(7, "36h11"),
        safe_approach_pose_object=PoseData.identity(),
    )
    definition = SimpleNamespace(get_routine=lambda routine_id: routine)
    editor = ProbeGeometryEditor(
        object_repository=SimpleNamespace(load=lambda object_id: definition),
        geometry=geometry,
        sensor_attachment_controller=_AttachmentController(
            active_transform
        ),
    )
    draft = SimpleNamespace(
        selected_object_id="object",
        selected_routine_id="routine_with_different_sensor",
    )

    result = editor._resolve_geometry(
        draft,
        "view",
        ImagePoint(u=10, v=20),
        "surface_fit",
        0.03,
        0.20,
    )

    assert geometry.hand_to_probe_pose == active_transform
    assert result.probe_setup.safe_approach_approved
    assert result.probe_setup.safe_approach_pose_object == routine.safe_approach_pose_object


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
