"""Manual final probe motions bypass the map without bypassing approach travel."""

import pytest

from fault_detector_spot.inspection.setup.probe_setup_motion import (
    ProbeMotionKind, ProbeMotionRequest,
)
from fault_detector_spot.shared.geometry.models import Vector3Data
from test_custom_probe_workflow import begin_custom, move
from test_probe_setup_coordinator import pose


@pytest.fixture
def custom_probe(tmp_path):
    probe, commands, state = begin_custom(tmp_path)
    state, _ = move(probe, commands, state, ProbeMotionKind.MOVE_SAFE_APPROACH)
    probe.motion_state_source.pose = pose(.7)
    state = probe.add_pathing_point(state.context, "approach waypoint", .01, .5)
    probe.motion_state_source.pose = pose(.8)
    state = probe.approve_aligned_pose(state.context)
    probe.motion_state_source.pose = pose(.9)
    state = probe.add_pathing_point(state.context, "final waypoint", .01, .2, "probe")
    probe.motion_state_source.pose = pose(1.)
    state = probe.approve_probe_pose(state.context)
    return probe, state


@pytest.mark.parametrize("stage", ["alignment", "probe"])
@pytest.mark.parametrize("kind", [
    ProbeMotionKind.MOVE_ALIGNED_PREAPPROACH,
    ProbeMotionKind.ADJUST_ALIGNED_PREAPPROACH,
    ProbeMotionKind.ADJUST_PATHING_POSE,
    ProbeMotionKind.MOVE_PATHING_POINT,
    ProbeMotionKind.MOVE_PRE_APPROACH_PATH,
])
def test_manual_final_and_alignment_segments_keep_separate_map_policy(custom_probe, kind, stage):
    probe, state = custom_probe
    operation = probe.prepare_motion(
        state.context,
        ProbeMotionRequest(kind, path_stage=stage, translation=Vector3Data(.01, 0., 0.)),
    )

    assert operation.request.command.ignore_environment_collisions is (stage == "probe")


@pytest.mark.parametrize("stage", ["alignment", "probe"])
def test_safe_approach_does_not_inherit_final_stage_bypass(custom_probe, stage):
    probe, state = custom_probe
    operation = probe.prepare_motion(
        state.context, ProbeMotionRequest(ProbeMotionKind.MOVE_SAFE_APPROACH, path_stage=stage),
    )

    assert operation.request.command.ignore_environment_collisions is False
