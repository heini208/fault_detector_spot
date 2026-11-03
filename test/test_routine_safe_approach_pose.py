"""Shared routine pose persistence, capture, and execution contracts."""

from dataclasses import replace
from types import SimpleNamespace

import pytest

from fault_detector_msgs.msg import OperationalIntent
from fault_detector_spot.inspection.execution.saved_probe_motion import saved_probe_command
from fault_detector_spot.inspection.model.models import InspectionObject
from fault_detector_spot.inspection.repository.object_repository import ObjectRepository
from fault_detector_spot.inspection.setup.probe_setup_motion import (
    ProbeMotionKind,
    ProbeMotionRequest,
    ProbeSetupMotionCommandFactory,
)
from fault_detector_spot.inspection.setup.probe_refinement_session import (
    RefinementStage,
)
from probe_execution_fixtures import inspection_object, pose, sensor
from test_probe_setup_coordinator import (
    coordinator, create_selected_routine,
)


@pytest.fixture
def saved_points(tmp_path):
    repository = ObjectRepository(tmp_path / "objects")
    repository.create(inspection_object())
    return repository, sensor()


def saved_safe_command(repository, attachment, point_id="point_1"):
    intent = OperationalIntent(
        intent=OperationalIntent.INTENT_MOVE_SAVED_PROBE_SAFE_APPROACH,
        object_id="motor_a", routine_id="scan", probe_point_id=point_id,
    )
    return saved_probe_command(
        intent, repository, None,
        SimpleNamespace(require_motion_attachment=lambda: attachment),
        ProbeSetupMotionCommandFactory(),
    )


def test_every_point_loads_the_latest_shared_pose_without_rewriting_points(saved_points):
    repository, attachment = saved_points
    definition = repository.load("motor_a")
    first = definition.routines[0].probe_points[0]
    repository.add_probe_point("motor_a", "scan", replace(first, probe_point_id="second"))
    before = repository.load("motor_a").to_dict()["routines"][0]["probe_points"]
    repository.set_routine_safe_approach_pose("motor_a", "scan", pose(x=0.75))
    after = repository.load("motor_a").to_dict()
    assert after["routines"][0]["probe_points"] == before
    assert all("safe_approach_pose_object" not in point for point in before)
    restored = InspectionObject.from_dict(after)
    assert restored.routines[0].require_safe_approach_pose() == pose(x=0.75)
    for point_id in ("point_1", "second"):
        command = saved_safe_command(repository, attachment, point_id)
        assert command.offset.position.x == 0.75


def test_old_per_point_pose_is_not_silently_chosen_for_routine(saved_points):
    repository, attachment = saved_points
    data = repository.load("motor_a").to_dict()
    routine = data["routines"][0]
    del routine["safe_approach_pose_object"]
    routine["probe_points"][0]["safe_approach_pose_object"] = pose(x=0.9).to_dict()
    repository.save(InspectionObject.from_dict(data))
    with pytest.raises(ValueError, match="routine safe pre-approach"):
        saved_safe_command(repository, attachment)


def test_capture_is_routine_scoped_and_needs_no_reference_pixel(tmp_path):
    probe, commands = coordinator(tmp_path)
    state = create_selected_routine(probe, probe.open_context("probe-ui").context)
    probe.motion_state_source.pose = pose(x=0.9)
    state = probe.save_routine_safe_approach_pose(state.context, .045)
    assert state.routine_safe_position_tolerance_m == .045
    assert state.has_routine_safe_approach_pose
    assert state.reference_pixel is None
    routine = probe.object_repository.load("motor").get_routine("magnetic_scan")
    assert routine.safe_approach_position_tolerance_m == .045
    assert routine.require_safe_approach_pose() == pose(x=0.9)
    assert commands.submitted == []
    commands.active = "busy"
    probe.motion_state_source.pose = pose(x=1.1)
    with pytest.raises(RuntimeError):
        probe.save_routine_safe_approach_pose(state.context)
    assert probe.object_repository.load("motor").get_routine(
        "magnetic_scan"
    ).require_safe_approach_pose() == pose(x=0.9)


def test_probe_workflow_cannot_author_a_private_safe_pose(tmp_path):
    probe, _ = coordinator(tmp_path)
    state = create_selected_routine(probe, probe.open_context("probe-ui").context)
    draft = probe._drafts[state.context.context_id]
    with pytest.raises(ValueError, match="routine setup"):
        probe.refinement_controller.approve(draft, RefinementStage.SAFE_APPROACH)
    with pytest.raises(ValueError, match="routine setup"):
        probe.refinement_controller.prepare_motion(
            state.context, draft,
            ProbeMotionRequest(kind=ProbeMotionKind.ADJUST_SAFE_APPROACH),
        )
