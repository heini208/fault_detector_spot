"""Regression guard for probe refinement extraction."""

import ast
from pathlib import Path


def _coordinator_source():
    package_root = Path(__file__).parents[1]
    path = (
        package_root
        / "fault_detector_spot/application/coordinators/"
        "probe_setup_coordinator.py"
    )
    return path.read_text(encoding="utf-8")


def test_probe_coordinator_delegates_refinement_motion_ownership():
    source = _coordinator_source()

    assert "ProbeRefinementController" in source
    assert "self._operations" not in source
    assert "def _motion_stage(" not in source
    assert "def _verify_achieved_motion(" not in source
    assert "def _absolute_motion_command(" not in source
    assert "def _relative_motion_command(" not in source
    assert "def _current_probe_pose(" not in source


def test_require_idle_remains_an_instance_method():
    tree = ast.parse(_coordinator_source())
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
    assert [argument.arg for argument in method.args.args[:2]] == [
        "self",
        "context",
    ]
