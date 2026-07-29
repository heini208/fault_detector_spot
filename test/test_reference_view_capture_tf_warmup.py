"""Tests for image-timestamp TF handling during reference capture."""

from fault_detector_spot.behaviour_tree.nodes.inspection import (
    capture_inspection_object_reference_view as capture_module,
)


def test_capture_behavior_does_not_preflight_latest_tf():
    assert not hasattr(
        capture_module.CaptureInspectionObjectReferenceView,
        "_camera_transform_ready",
    )
    assert not hasattr(
        capture_module.CaptureInspectionObjectReferenceView,
        "_wait_for_input_warmup",
    )
