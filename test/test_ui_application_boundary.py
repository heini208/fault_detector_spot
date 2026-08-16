"""Guard the remote UI application boundary."""

from pathlib import Path


UI_ROOT = Path(__file__).parents[1] / "fault_detector_spot" / "ui"
OPERATIONAL_UI_FILES = (
    UI_ROOT / "fault_detector_ui.py",
    UI_ROOT / "manipulation" / "controls.py",
    UI_ROOT / "navigation" / "base_movement_controls.py",
    UI_ROOT / "navigation" / "controls.py",
    UI_ROOT / "inspection" / "controls.py",
    UI_ROOT / "sensor" / "controls.py",
)


def test_operational_ui_does_not_use_internal_command_messages():
    forbidden = (
        "BasicCommand",
        "ComplexCommand",
        "CommandStatus",
        "build_basic_command",
        "complex_command_publisher",
    )

    for path in OPERATIONAL_UI_FILES:
        source = path.read_text(encoding="utf-8")
        for value in forbidden:
            assert value not in source, f"{value} remains in {path.name}"


def test_operational_ui_does_not_reference_internal_command_topics():
    forbidden_topics = (
        "fault_detector/commands/basic_command",
        "fault_detector/commands/complex_command",
        "fault_detector/_internal/commands",
        "fault_detector/_internal/command_status",
        "fault_detector/command_status",
        "fault_detector/command_tree_status",
    )

    for path in OPERATIONAL_UI_FILES:
        source = path.read_text(encoding="utf-8")
        for topic in forbidden_topics:
            assert topic not in source, f"{topic} remains in {path.name}"


def test_navigation_ui_does_not_own_setup_persistence_or_topics():
    path = UI_ROOT / "navigation" / "controls.py"
    source = path.read_text(encoding="utf-8")
    forbidden = (
        "MapRepository",
        "RTABHelper",
        "ComplexCommand",
        "'/map_list'",
        "'/waypoint_list'",
        "'/landmark_list'",
        "'/active_map'",
        "show_setup_unavailable",
    )

    for value in forbidden:
        assert value not in source, f"{value} remains in {path.name}"


def test_inspection_ui_only_renders_probe_setup_state_and_intent():
    path = UI_ROOT / "inspection" / "controls.py"
    source = path.read_text(encoding="utf-8")
    forbidden = (
        "ObjectRepository",
        "MultiReferenceViewRepository",
        "self.object_repository",
        "self.reference_view_repository",
        "project_reference_pixel",
        "estimate_reference_surface_normal",
        "resolve_reference_approach_direction",
        "resolve_reference_surface_target",
        "initialize_reference_probe_setup",
        "ProbePoint(",
    )

    for value in forbidden:
        assert value not in source, f"{value} remains in {path.name}"

    assert "ProbeSetupIntent" in source
    assert "ProbeSetupMotionIntent" in source
    assert "apply_setup_state" in source
    assert "apply_reference_preview" in source
    assert "_complete_pending_refinement_motion" not in source
    assert "_fail_pending_refinement_motion" not in source


def test_probe_setup_ui_adapter_does_not_reconstruct_domain_session():
    path = UI_ROOT / "ros" / "probe_setup_state_adapter.py"
    source = path.read_text(encoding="utf-8")

    assert "ProbeRefinementSession" not in source
    assert "PendingRefinementMotion" not in source
    assert "ProbeRefinementSnapshot" in source
    assert "MappingProxyType" in source


def test_inspection_refinement_state_is_presentation_only():
    paths = (
        UI_ROOT / "inspection" / "controls.py",
        UI_ROOT / "inspection" / "finalizing_controls.py",
    )
    source = "".join(path.read_text(encoding="utf-8") for path in paths)

    assert "self._refinement_session" not in source
    assert "_refinement_workflow_active" not in source
    assert "_require_refinement_session" not in source
    assert "_synchronize_refinement_session" not in source
    assert "_handle_surface_tolerance_changed" not in source
    assert "surface_distance_verified = False" not in source
    assert "_refinement_presentation" in source
    assert "_require_refinement_presentation" in source
    assert "_synchronize_refinement_presentation" in source


def test_behaviour_tree_has_no_navigation_authoring_commands():
    root = UI_ROOT.parents[0]
    runner = root / "application" / "behaviour_tree" / "runner.py"
    command_ids = root / "application" / "commanding" / "command_ids.py"
    source = runner.read_text(encoding="utf-8") + command_ids.read_text(
        encoding="utf-8"
    )
    forbidden = (
        "CREATE_MAP",
        "DELETE_MAP",
        "ADD_CURRENT_POSITION_WAYPOINT",
        "ADD_TAG_AS_LANDMARK",
        "DELETE_WAYPOINT",
        "DELETE_LANDMARK",
        "AddGoalPoseAsWaypoint",
        "AddGoalPoseAsLandmark",
    )

    for value in forbidden:
        assert value not in source


def test_legacy_navigation_state_topics_are_removed():
    root = UI_ROOT.parents[0]
    paths = (
        root / "mapping" / "runtime" / "rtab_helper.py",
        root / "application" / "behaviour_tree" / "runner.py",
    )
    source = "".join(path.read_text(encoding="utf-8") for path in paths)

    for topic in ("map_list", "waypoint_list", "landmark_list"):
        assert topic not in source
