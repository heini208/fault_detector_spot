"""Regression guards for presentation-only inspection command state."""

import ast
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock


ROOT = Path(__file__).parents[1]
CONTROLS = ROOT / "fault_detector_spot/ui/inspection/controls.py"


def test_inspection_ui_does_not_own_application_command_state():
    source = CONTROLS.read_text(encoding="utf-8")

    forbidden = (
        "ApplicationCommandState",
        "PROBE_MOTION_SETTLE_SEC",
        "_command_state",
        "_buffered_command_count",
        "_last_command_completion_monotonic",
        "_require_command_path_idle",
        "_probe_motion_pending",
        "time.monotonic",
    )
    for value in forbidden:
        assert value not in source


def test_application_state_hook_only_forwards_to_navigation_presentation():
    tree = ast.parse(CONTROLS.read_text(encoding="utf-8"))
    method = next(
        node for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef)
        and node.name == "handle_application_state"
    )
    # Exercise the hook without constructing Qt widgets or importing ROS.
    namespace = {}
    code = compile(
        ast.Module(body=[method], type_ignores=[]), str(CONTROLS), "exec",
    )
    exec(code, namespace)
    navigation = Mock()
    controls = SimpleNamespace(routine_navigation_controls=navigation)
    status = object()

    namespace[method.name](controls, status)

    navigation.handle_application_state.assert_called_once_with(status)
    assert vars(controls) == {"routine_navigation_controls": navigation}
