"""Lock the final setup coordinator ownership boundaries."""

import inspect
from pathlib import Path

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.coordinators.probe_reference_capture_coordinator import (
    ProbeReferenceCaptureCoordinator,
)
from fault_detector_spot.ui.inspection.finalizing_controls import (
    FinalizingInspectionControls,
)


ROOT = Path(__file__).parents[1] / "fault_detector_spot"


def test_nonphysical_inspection_setup_ids_are_not_commands():
    names = {value.name for value in CommandID}
    assert "CREATE_INSPECTION_OBJECT" not in names
    assert "CREATE_INSPECTION_ROUTINE" not in names
    assert "DELETE_INSPECTION_OBJECT" not in names
    assert "DELETE_INSPECTION_ROUTINE" not in names
    assert "CAPTURE_INSPECTION_OBJECT_REFERENCE_VIEW" not in names


def test_behaviour_tree_has_no_inspection_authoring_execution():
    source = (
        ROOT / "application/behaviour_tree/runner.py"
    ).read_text(encoding="utf-8")
    forbidden = (
        "CreateInspectionDefinition",
        "DeleteInspectionDefinition",
        "CaptureInspectionObjectReferenceView",
        "CREATE_INSPECTION_OBJECT",
        "CREATE_INSPECTION_ROUTINE",
        "DELETE_INSPECTION_OBJECT",
        "DELETE_INSPECTION_ROUTINE",
        "CAPTURE_INSPECTION_OBJECT_REFERENCE_VIEW",
    )
    for value in forbidden:
        assert value not in source


def test_reference_capture_is_application_coordinator_not_bt_behavior():
    source = inspect.getsource(ProbeReferenceCaptureCoordinator)
    assert "py_trees" not in source
    assert "CommandID" not in source
    assert "capture_reference_views" in source
    assert "reference_tag(" in source
    assert "_require_command_lane_idle" in source
    assert "_release_synchronizers" in source


def test_operational_inspection_controls_do_not_create_tf_or_depth_subscriptions():
    source = inspect.getsource(
        FinalizingInspectionControls.init_ros_communication
    )
    assert "tf2_ros" not in source
    assert "hand_depth" not in source
    assert "depth_registered" not in source
    assert "base_tags" not in source


def test_reference_capture_ui_only_delegates_to_application_boundary():
    source = inspect.getsource(
        FinalizingInspectionControls.handle_capture_reference_view
    )
    assert "execute_probe_reference_capture" in source
    assert "capture_reference_views(" not in source
    assert "MultiReferenceViewRepository" not in source
    assert "tf_buffer" not in source


def test_ui_root_does_not_subscribe_to_base_tags_for_setup_authority():
    source = (
        ROOT / "ui/fault_detector_ui.py"
    ).read_text(encoding="utf-8")
    assert "fault_detector/state/base_tags" not in source
    assert "_process_base_tags" not in source
