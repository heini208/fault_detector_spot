"""Ownership guards for standalone move-close-to-surface execution."""

import ast
from pathlib import Path

from fault_detector_spot.inspection.setup.probe_refinement_session import (
    PendingRefinementMotion,
    ProbeRefinementSession,
)
from fault_detector_spot.inspection.setup.probe_setup_motion import (
    ProbeMotionKind,
)


ROOT = Path(__file__).parents[1] / "fault_detector_spot"


def source(relative):
    return (ROOT / relative).read_text(encoding="utf-8")


def test_probe_setup_motion_contains_only_safe_and_aligned_primitives():
    assert {kind.value for kind in ProbeMotionKind} == {
        "move_safe_approach",
        "move_aligned_preapproach",
        "adjust_safe_approach",
        "adjust_aligned_preapproach",
    }


def test_refinement_session_contains_no_close_surface_execution_state():
    pending_fields = PendingRefinementMotion.__dataclass_fields__
    session_fields = ProbeRefinementSession.__dataclass_fields__

    assert "axial_correction_m" not in pending_fields
    assert "surface_distance_verified" not in session_fields
    assert "cumulative_inward_travel_m" not in session_fields
    assert not hasattr(ProbeRefinementSession, "mark_surface_verified")
    assert not hasattr(ProbeRefinementSession, "approve_verified_probe")


def test_finalization_names_derived_geometry_without_verification_claims():
    controller = source(
        "application/coordinators/probe_finalization_controller.py"
    )
    coordinator = source(
        "application/coordinators/probe_setup_coordinator.py"
    )
    runner = source(
        "application/coordinators/"
        "probe_refinement_finalization_coordinator.py"
    )

    assert "def approve_probe_geometry(" in controller
    assert "def approve_probe_geometry_for_finalization(" in coordinator
    assert "approve_probe_geometry_for_finalization" in runner
    assert "approve_verified_probe" not in controller
    assert "approve_verified_probe" not in coordinator
    assert "approve_verified_probe" not in runner


def test_close_surface_operation_stays_outside_probe_setup_transport():
    ui = source("ui/fault_detector_ui.py")
    controls = source("ui/inspection/controls.py")
    setup_client = source("ui/ros/probe_setup_client.py")
    state_adapter = source("ui/ros/probe_setup_state_adapter.py")
    application_api = source("application/api/application_api_node.py")
    motion_api = source("application/api/probe_setup_motion_api.py")

    assert "def execute_move_close_to_surface(" in ui
    assert "INTENT_MOVE_CLOSE_TO_SURFACE" in ui
    assert "target_surface_distance_m" in ui
    assert "execute_operation(intent)" in ui
    assert "execute_move_close_to_surface" in controls
    assert "surface_verification" not in controls
    assert "surface_verification" not in setup_client
    assert "surface_distance_verified" not in state_adapter
    assert "surface_verification" not in state_adapter
    assert "ProbeSurfaceVerificationApi" not in application_api
    assert "ExecuteOperation" in application_api
    assert "ADJUST_PROBE_DISTANCE" not in motion_api


def test_obsolete_setup_verification_modules_are_absent():
    obsolete = (
        ROOT
        / "application/coordinators/probe_surface_verification_controller.py",
        ROOT
        / "application/coordinators/probe_surface_verification_runner.py",
        ROOT / "inspection/setup/probe_surface_verification.py",
    )

    assert all(not path.exists() for path in obsolete)


def test_close_surface_runtime_remains_independent_of_probe_setup():
    runtime = source(
        "inspection/execution/probe_surface_runtime_state.py"
    )

    assert "surface_distance_samples" in runtime
    assert "active_attachment" in runtime
    assert "probe_setup" not in runtime


def test_finalization_uses_generic_close_surface_result():
    controls = source("ui/inspection/finalizing_controls.py")

    assert "_surface_result_current" in controls
    assert "INTENT_MOVE_CLOSE_TO_SURFACE" in controls
    assert "SURFACE_VERIFICATION_CONVERGED" not in controls


def test_reference_probe_setup_uses_shared_pose_geometry():
    path = ROOT / "inspection/setup/reference_probe_setup.py"
    text = path.read_text(encoding="utf-8")
    tree = ast.parse(text)
    local_functions = {
        node.name
        for node in tree.body
        if isinstance(node, ast.FunctionDef)
    }
    generic_helpers = {
        "add_vectors",
        "compose_poses",
        "inverse_pose",
        "probe_pose_to_hand_pose",
        "relative_pose",
        "scale_vector",
        "subtract_vectors",
    }

    assert generic_helpers.isdisjoint(local_functions)
    assert "from fault_detector_spot.inspection.geometry.pose import (" in text
