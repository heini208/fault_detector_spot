"""Regression guards for probe setup client workflow admission."""

import ast
from pathlib import Path


def _source():
    package_root = Path(__file__).parents[1]
    return (
        package_root
        / "fault_detector_spot/ui/ros/probe_setup_client.py"
    ).read_text(encoding="utf-8")


def _method(name):
    tree = ast.parse(_source())
    client = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef)
        and node.name == "ProbeSetupClient"
    )
    return next(
        node
        for node in client.body
        if isinstance(node, ast.FunctionDef)
        and node.name == name
    )


def _attributes(method):
    return {
        node.attr
        for node in ast.walk(method)
        if isinstance(node, ast.Attribute)
    }


def test_obsolete_surface_verification_action_is_not_owned_by_setup_client():
    source = _source()

    assert "ExecuteProbeSurfaceVerification" not in source
    assert "execute_surface_verification" not in source
    assert "_surface_client" not in source
    assert "_surface_goal_handles" not in source


def test_setup_transactions_are_not_locked_by_async_action_bookkeeping():
    attributes = _attributes(_method("_send"))

    assert "_pending_request_id" in attributes
    assert "_motion_goal_handles" not in attributes
    assert "_finalization_goal_handles" not in attributes
    assert "_capture_goal_handles" not in attributes


def test_reference_capture_only_uses_its_own_local_action_lock():
    attributes = _attributes(_method("capture_reference_views"))

    assert "_pending_request_id" in attributes
    assert "_capture_goal_handles" in attributes
    assert "_motion_goal_handles" not in attributes
    assert "_finalization_goal_handles" not in attributes
