"""Regression guards for the alignment-orientation command schema."""

from fault_detector_msgs.msg import ProbeSetupMotionIntent


def test_surface_orientation_has_one_explicit_motion_operation():
    assert ProbeSetupMotionIntent.OPERATION_ORIENT_TO_SURFACE == 5
    assert ProbeSetupMotionIntent.OPERATION_ORIENT_TO_TAG == 6


def test_alignment_orientation_has_explicit_workflow_state():
    from fault_detector_msgs.msg import ProbeSetupState

    assert ProbeSetupState.MOTION_ORIENTED == 4
