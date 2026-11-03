"""Recovery keeps retraction available once pending movement has settled."""

import pytest

from test_probe_surface_navigation import application, wizard


@pytest.mark.parametrize("recovery_source", ["setup", "distance"])
def test_emergency_recovery_keeps_retract_available(wizard, recovery_source):
    controls, state, *_ = wizard
    # Recovery must allow retreat even when alignment is no longer reached.
    state.alignment_motion_state = state.MOTION_NOT_TESTED
    state.refinement_recovery_required = recovery_source == "setup"
    controls.apply_setup_state(state)
    controls._distance_failure_requires_retraction = recovery_source == "distance"
    controls._refresh_refinement_dialog()
    assert controls.retract_without_saving_button.isEnabled()

    state.motion_pending = True
    controls.apply_setup_state(state)
    controls._distance_failure_requires_retraction = recovery_source == "distance"
    controls._refresh_refinement_dialog()
    assert not controls.retract_without_saving_button.isEnabled()
