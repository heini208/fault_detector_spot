"""Regression guard for final probe setup split."""

import ast
from pathlib import Path


def _source():
    package_root = Path(__file__).parents[1]
    return (
        package_root
        / "fault_detector_spot/application/coordinators/"
        "probe_setup_coordinator.py"
    ).read_text(encoding="utf-8")


def test_probe_setup_delegates_only_refinement_and_finalization_state():
    source = _source()

    assert "ProbeSurfaceVerificationController" not in source
    assert "ProbeSurfaceVerificationRunner" not in source
    assert "ProbeFinalizationController" in source
    assert "_finalizations" not in source
    assert "def _persist_probe_point(" not in source
    assert "def _surface_verification_session(" not in source
    assert "def _require_finalization(" not in source


def test_require_idle_is_still_an_instance_method():
    tree = ast.parse(_source())
    coordinator = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef)
        and node.name == "ProbeSetupCoordinator"
    )
    method = next(
        node
        for node in coordinator.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "_require_idle"
    )

    assert method.decorator_list == []
    assert [argument.arg for argument in method.args.args] == [
        "self",
        "context",
        "finalization_request_id",
    ]
