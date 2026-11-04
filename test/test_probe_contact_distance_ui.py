"""Contact targets use the same distance contract in UI and domain logic."""

import pytest

from fault_detector_msgs.msg import ProbeSetupIntent
from fault_detector_spot.ui.inspection.finalizing_controls import (
    FinalizingInspectionControls,
)
from test_probe_point_guided_workflow import FakeUI, application as application
from test_probe_safe_approach_navigation import safe_state


@pytest.fixture
def controls(application):
    value = FinalizingInspectionControls(FakeUI())
    value.apply_setup_state(safe_state())
    yield value
    value.refinement_dialog.hide()


def test_reference_contact_target_submits_geometry_at_minimum_clearance(controls):
    controls.reference_target_distance_field.setText("0.000")
    controls.reference_preapproach_distance_field.setText("0.050")

    assert controls.reference_target_distance_field.hasAcceptableInput()
    controls.reference_target_distance_field.editingFinished.emit()

    intent = controls.ui.requests[-1]
    assert intent.operation == ProbeSetupIntent.OPERATION_UPDATE_GEOMETRY
    assert intent.target_surface_distance_m == 0.0
    assert intent.aligned_preapproach_distance_m == pytest.approx(0.05)


def test_dialog_contact_commit_submits_geometry_and_waits_for_state(controls):
    dialog = controls.refinement_dialog
    dialog.target_distance_field.setText("0.000")
    dialog.aligned_distance_field.setText("0.050")
    dialog.target_distance_field.setModified(True)

    assert dialog.target_distance_field.hasAcceptableInput()
    dialog.target_distance_field.editingFinished.emit()

    intent = controls.ui.requests[-1]
    assert intent.operation == ProbeSetupIntent.OPERATION_UPDATE_GEOMETRY
    assert intent.target_surface_distance_m == 0.0
    assert intent.aligned_preapproach_distance_m == pytest.approx(0.05)
    assert controls.reference_target_distance_field.text() == "0.000"
    assert not dialog.target_distance_field.isModified()
    assert not dialog.target_distance_field.isEnabled()
    assert not controls.use_current_alignment_button.isEnabled()


def test_reference_selection_rejects_invalid_clearance_without_raising(
    controls, monkeypatch,
):
    warnings = []
    monkeypatch.setattr(controls, "show_warning", lambda *args: warnings.append(args))
    controls._reference_slot_view_ids[0] = "hand-view"
    controls.reference_target_distance_field.setText("0.000")
    controls.reference_preapproach_distance_field.setText("0.049")

    controls._handle_reference_slot_point_changed(0, 20, 30)

    assert not controls.ui.requests
    assert "at least 0.05 m" in warnings[-1][1]


@pytest.mark.parametrize("target, aligned", [
    ("-0.001", "0.20"),
    ("nan", "0.20"),
    ("inf", "0.20"),
    ("0", "0"),
    ("0", "nan"),
    ("0", "inf"),
    ("0", "0.049"),
    ("0.10", "0.149"),
])
def test_reference_invalid_distances_do_not_submit_geometry(controls, target, aligned):
    controls.reference_target_distance_field.setText(target)
    controls.reference_preapproach_distance_field.setText(aligned)

    assert controls._request_geometry_update() is False
    assert not controls.ui.requests


@pytest.mark.parametrize("target, aligned", [
    ("-0.001", "0.20"),
    ("nan", "0.20"),
    ("inf", "0.20"),
    ("0", "0"),
    ("0", "nan"),
    ("0", "inf"),
    ("0", "0.049"),
    ("0.10", "0.149"),
])
def test_dialog_invalid_commit_restores_previous_distances(controls, target, aligned):
    dialog = controls.refinement_dialog
    dialog.target_distance_field.setText(target)
    dialog.aligned_distance_field.setText(aligned)
    dialog.target_distance_field.setModified(True)

    dialog.target_distance_field.editingFinished.emit()

    assert not controls.ui.requests
    assert dialog.target_distance_field.text() == "0.100"
    assert dialog.aligned_distance_field.text() == "0.200"
    assert controls.refinement_recovery_status_label.text()


@pytest.mark.parametrize("text", ["0", "-0.001", "nan", "inf"])
def test_positive_tolerances_do_not_inherit_contact_target_rules(controls, text):
    field = controls.routine_safe_tolerance_field
    field.setText(text)

    assert not field.hasAcceptableInput()
    with pytest.raises(ValueError, match="positive and finite"):
        controls._distance_value(field, "Position tolerance")
