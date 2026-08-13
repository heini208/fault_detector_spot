"""Validate retryable base-tag readiness during reference capture."""

import pytest

from fault_detector_spot.application.coordinators.probe_reference_capture_coordinator import (
    ProbeReferenceCaptureCoordinator,
)
from fault_detector_spot.inspection.setup.reference_view_validation import (
    ReferenceViewCaptureNotReady,
)


class NotReadyStateSource:
    """Report transient base-tag stabilization failure."""

    def reference_tag(self, tag_id):
        raise ValueError(
            "Need at least 3 distinct base-tag observations; distinct=2"
        )


class ReadyStateSource:
    """Return an already stable base tag."""

    def __init__(self, tag):
        self.tag = tag

    def reference_tag(self, tag_id):
        return self.tag


def coordinator_with_state_source(state_source):
    coordinator = object.__new__(ProbeReferenceCaptureCoordinator)
    coordinator.motion_state_source = state_source
    return coordinator


def test_stable_tag_value_error_becomes_retryable_capture_readiness():
    coordinator = coordinator_with_state_source(NotReadyStateSource())

    with pytest.raises(
        ReferenceViewCaptureNotReady,
        match="distinct=2",
    ):
        coordinator._stable_reference_tag(7)


def test_stable_tag_success_is_passed_through():
    tag = object()
    coordinator = coordinator_with_state_source(ReadyStateSource(tag))

    assert coordinator._stable_reference_tag(7) is tag
