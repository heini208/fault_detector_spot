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


class HistoryStateSource:
    """Return capture-window tag observations."""

    def __init__(self, tags):
        self.tags = tags

    def reference_tag_history(
        self,
        tag_id,
        receipt_not_before,
        receipt_not_after,
    ):
        return self.tags


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


def test_empty_historical_tag_window_is_retryable():
    coordinator = coordinator_with_state_source(HistoryStateSource(()))

    with pytest.raises(
        ReferenceViewCaptureNotReady,
        match="during the reference capture window",
    ):
        coordinator._historical_reference_tags(
            7,
            10.0,
            11.0,
        )
