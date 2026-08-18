"""Contracts for the final library cleanup."""

from pathlib import Path

from fault_detector_spot.inspection.geometry.pose import (
    compose_poses,
    inverse_pose,
    probe_pose_to_hand_pose,
)
from fault_detector_spot.inspection.model.models import PoseData


ROOT = Path(__file__).parents[1] / "fault_detector_spot"


def source(relative):
    return (ROOT / relative).read_text(encoding="utf-8")


def test_base_actions_no_longer_use_tf_transformations():
    for relative in (
        "navigation/behaviours/move_base/base_move_to_tag_action.py",
        "navigation/behaviours/move_base/base_move_relative_action.py",
    ):
        text = source(relative)
        assert "tf_transformations" not in text
        assert "quaternion_to_rpy" in text


def test_execution_geometry_no_longer_depends_on_setup_module():
    text = source("inspection/execution/probe_execution_target.py")
    assert "inspection.setup.reference_probe_setup" not in text
    assert "inspection.geometry.pose" in text


def test_refinement_controller_does_not_create_ghost_verification_state():
    text = source("application/coordinators/probe_refinement_controller.py")
    assert "draft.surface_verification" not in text


def test_pose_data_helpers_preserve_identity_geometry():
    identity = PoseData.identity()

    assert compose_poses(identity, identity) == identity
    assert inverse_pose(identity) == identity
    assert probe_pose_to_hand_pose(identity, identity) == identity
