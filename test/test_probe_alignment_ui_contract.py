"""Regression guards for the alignment-orientation command schema."""

from fault_detector_msgs.msg import ProbeSetupMotionIntent


def test_surface_orientation_has_one_explicit_motion_operation():
    assert ProbeSetupMotionIntent.OPERATION_ORIENT_TO_SURFACE == 5
    assert ProbeSetupMotionIntent.OPERATION_ORIENT_TO_TAG == 6


def test_alignment_orientation_has_explicit_workflow_state():
    from fault_detector_msgs.msg import ProbeSetupState

    assert ProbeSetupState.MOTION_ORIENTED == 4


def test_legacy_calculated_orientation_payload_is_removed():
    fields = ProbeSetupMotionIntent.get_fields_and_field_types()
    assert "alignment_orientation_mode" not in fields
    assert "orientation_only" not in fields
    assert "has_calculated_surface_orientation" not in fields
    assert "calculated_surface_orientation_object" not in fields
