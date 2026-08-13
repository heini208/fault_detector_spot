"""Tests for explicit alignment orientation motion requests."""

import pytest

from fault_detector_spot.inspection.model.models import QuaternionData
from fault_detector_spot.inspection.setup.probe_setup_motion import (
    ProbeAlignmentOrientationMode,
    ProbeMotionKind,
    ProbeMotionRequest,
)


def test_alignment_defaults_to_tag_orientation():
    request = ProbeMotionRequest(
        kind=ProbeMotionKind.MOVE_ALIGNED_PREAPPROACH
    )

    request.validate()

    assert request.alignment_orientation_mode is (
        ProbeAlignmentOrientationMode.TAG
    )
    assert not request.orientation_only


def test_calculated_surface_alignment_requires_orientation():
    request = ProbeMotionRequest(
        kind=ProbeMotionKind.MOVE_ALIGNED_PREAPPROACH,
        alignment_orientation_mode=(
            ProbeAlignmentOrientationMode.CALCULATED_SURFACE
        ),
    )

    with pytest.raises(ValueError, match="requires an orientation"):
        request.validate()


def test_calculated_surface_alignment_accepts_explicit_orientation():
    request = ProbeMotionRequest(
        kind=ProbeMotionKind.MOVE_ALIGNED_PREAPPROACH,
        alignment_orientation_mode=(
            ProbeAlignmentOrientationMode.CALCULATED_SURFACE
        ),
        calculated_surface_orientation_object=QuaternionData.identity(),
        orientation_only=True,
    )

    request.validate()


def test_orientation_only_is_rejected_outside_alignment():
    request = ProbeMotionRequest(
        kind=ProbeMotionKind.MOVE_SAFE_APPROACH,
        orientation_only=True,
    )

    with pytest.raises(ValueError, match="only for alignment"):
        request.validate()
