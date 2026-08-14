"""Tests for end-effector force baselining and probe-axis projection."""

import pytest

from fault_detector_spot.inspection.model.models import Vector3Data
from fault_detector_spot.inspection.sensing.end_effector_force import (
    EndEffectorForceSample,
    estimate_force_baseline,
    project_probe_force_delta,
)


def sample(x, y, z, receipt_time, frame_id="hand"):
    return EndEffectorForceSample(
        force_hand=Vector3Data(x=x, y=y, z=z),
        stamp_seconds=10.0 + receipt_time,
        receipt_time=receipt_time,
        frame_id=frame_id,
    )


def test_baseline_uses_component_medians_and_reports_noise_span():
    samples = [
        sample(1.0, -2.0, 3.0, 0.00),
        sample(1.2, -1.8, 3.1, 0.10),
        sample(0.9, -2.1, 2.9, 0.20),
    ]

    baseline = estimate_force_baseline(
        samples,
        minimum_samples=3,
        minimum_span_sec=0.20,
    )

    assert baseline.force_hand.x == pytest.approx(1.0)
    assert baseline.force_hand.y == pytest.approx(-2.0)
    assert baseline.force_hand.z == pytest.approx(3.0)
    assert baseline.sample_count == 3
    assert baseline.sample_span_sec == pytest.approx(0.20)
    assert baseline.maximum_component_span_n == pytest.approx(0.3)


def test_force_delta_projects_onto_normalized_probe_axis():
    baseline = estimate_force_baseline(
        [
            sample(1.0, 2.0, 3.0, 0.00),
            sample(1.0, 2.0, 3.0, 0.10),
            sample(1.0, 2.0, 3.0, 0.20),
        ],
        minimum_samples=3,
        minimum_span_sec=0.20,
    )
    current = sample(4.0, 6.0, 3.0, 0.30)

    result = project_probe_force_delta(
        current,
        baseline,
        Vector3Data(x=2.0, y=0.0, z=0.0),
    )

    assert result.delta_force_hand == Vector3Data(x=3.0, y=4.0, z=0.0)
    assert result.axial_force_n == pytest.approx(3.0)
    assert result.lateral_force_n == pytest.approx(4.0)
    assert result.total_force_n == pytest.approx(5.0)


def test_baseline_rejects_mixed_frames_and_short_windows():
    with pytest.raises(ValueError, match="share one frame"):
        estimate_force_baseline(
            [
                sample(0.0, 0.0, 0.0, 0.00),
                sample(0.0, 0.0, 0.0, 0.10),
                sample(0.0, 0.0, 0.0, 0.20, frame_id="spot/hand"),
            ],
            minimum_samples=3,
            minimum_span_sec=0.20,
        )

    with pytest.raises(ValueError, match="span enough time"):
        estimate_force_baseline(
            [
                sample(0.0, 0.0, 0.0, 0.00),
                sample(0.0, 0.0, 0.0, 0.05),
                sample(0.0, 0.0, 0.0, 0.10),
            ],
            minimum_samples=3,
            minimum_span_sec=0.20,
        )


def test_projection_rejects_zero_probe_axis():
    baseline = estimate_force_baseline(
        [
            sample(0.0, 0.0, 0.0, 0.00),
            sample(0.0, 0.0, 0.0, 0.10),
            sample(0.0, 0.0, 0.0, 0.20),
        ],
        minimum_samples=3,
        minimum_span_sec=0.20,
    )

    with pytest.raises(ValueError, match="non-zero"):
        project_probe_force_delta(
            sample(1.0, 0.0, 0.0, 0.30),
            baseline,
            Vector3Data.zero(),
        )


def test_baseline_rejects_force_changes_that_indicate_arm_motion():
    moving = [
        sample(0.0, 0.0, 0.0, 0.00),
        sample(0.1, 0.0, 0.0, 0.10),
        sample(2.0, 0.0, 0.0, 0.20),
    ]

    with pytest.raises(ValueError, match="not stationary enough"):
        estimate_force_baseline(
            moving,
            minimum_samples=3,
            minimum_span_sec=0.20,
            maximum_allowed_component_span_n=1.0,
        )
