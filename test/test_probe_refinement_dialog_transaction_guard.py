"""Regression guards for refinement distance focus handling."""

import inspect

from fault_detector_spot.ui.inspection.probe_refinement_dialog import (
    ProbeRefinementDialog,
)


def test_unchanged_distance_focus_loss_does_not_submit_transaction():
    source = inspect.getsource(
        ProbeRefinementDialog._handle_distance_editing_finished
    )

    assert "isModified()" in source
    assert "_handle_dialog_distances_changed" in source


def test_changed_distance_commit_blocks_immediate_alignment_approval():
    source = inspect.getsource(
        ProbeRefinementDialog._handle_distance_editing_finished
    )

    assert "use_current_alignment_button" in source
    assert "setEnabled(False)" in source
