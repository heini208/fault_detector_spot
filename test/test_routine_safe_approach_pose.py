"""Shared routine pose persistence, capture, and execution contracts."""

from dataclasses import replace

import pytest

from fault_detector_spot.inspection.execution.probe_execution_session import (
    ProbeExecutionConfiguration,
)
from fault_detector_spot.inspection.model.models import InspectionObject
from fault_detector_spot.inspection.setup.probe_setup_motion import (
    ProbeMotionKind,
    ProbeMotionRequest,
)
from fault_detector_spot.inspection.setup.probe_refinement_session import (
    RefinementStage,
)
from test_probe_execution_session import repository_and_sensor, pose
from test_probe_setup_coordinator import (
    coordinator, create_selected_routine,
)


def test_every_point_loads_the_latest_shared_pose_without_rewriting_points(tmp_path):
    repository, attachment = repository_and_sensor(tmp_path)
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
        config = ProbeExecutionConfiguration.load(
            "motor_a", "scan", point_id, repository, attachment,
        )
        assert config.safe_approach_pose_object.to_pose() == pose(x=0.75)


def test_old_per_point_pose_is_not_silently_chosen_for_routine(tmp_path):
    repository, attachment = repository_and_sensor(tmp_path)
    data = repository.load("motor_a").to_dict()
    routine = data["routines"][0]
    del routine["safe_approach_pose_object"]
    routine["probe_points"][0]["safe_approach_pose_object"] = pose(x=0.9).to_dict()
    repository.save(InspectionObject.from_dict(data))
    with pytest.raises(ValueError, match="routine safe pre-approach"):
        ProbeExecutionConfiguration.load("motor_a", "scan", "point_1", repository, attachment)


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
