"""Depth-fit reuse must preserve calibration, TF and receipt freshness."""

from copy import deepcopy
from types import SimpleNamespace

import pytest

from fault_detector_msgs.msg import SensorAttachmentState
from fault_detector_spot.inspection.sensing import probe_surface_source
from fault_detector_spot.inspection.sensing.probe_surface_source import (
    HAND_DEPTH_HISTORY_MAX_SAMPLES,
    ProbeSurfaceSource,
)
from test_live_surface_distance import (
    distance_sample,
    planar_depth,
    probe_to_camera_pose,
)


@pytest.fixture
def harness(monkeypatch):
    state = SimpleNamespace(
        now=100.0, fit_calls=[], tf_calls=[], fit_delay=0.0,
        fit_error=None, tf_error=None, pose=probe_to_camera_pose(),
    )
    monkeypatch.setattr(probe_surface_source.time, "monotonic", lambda: state.now)
    node = SimpleNamespace(
        create_subscription=lambda *args, **kwargs: object(),
        destroy_subscription=lambda subscription: None,
    )
    state.source = ProbeSurfaceSource(node, None)
    _, state.camera = planar_depth()
    state.source._receive_hand_depth_camera_info(state.camera)
    state.attachment = SensorAttachmentState()
    state.attachment.status = SensorAttachmentState.STATUS_ACTIVE
    state.attachment.active_sensor_id = "hall_probe"
    state.attachment.attachment_revision = 1
    state.source._receive_attachment_state(state.attachment)

    def lookup(target, source, lookup_time):
        state.tf_calls.append((target, source, lookup_time.nanoseconds))
        if state.tf_error:
            raise ValueError(state.tf_error)
        return state.pose

    def measure(image, camera_info, pose):
        state.fit_calls.append((image, deepcopy(camera_info), deepcopy(pose)))
        state.now += state.fit_delay
        if state.fit_error:
            raise ValueError(state.fit_error)
        return distance_sample(
            0.20, image.header.stamp.sec + image.header.stamp.nanosec * 1e-9,
        )

    state.source._lookup_pose = lookup
    monkeypatch.setattr(probe_surface_source, "measure_probe_surface_distance", measure)

    def receive(stamp=10):
        image, _ = planar_depth()
        image.header.stamp.sec = stamp
        state.source._receive_hand_depth(image)

    state.receive = receive
    state.sample = lambda **kwargs: state.source.surface_distance_samples(
        "hall_probe", **kwargs,
    )
    return state


def test_repeated_poll_reuses_fits_but_checks_each_timestamped_tf(harness):
    for stamp in (10, 11, 12):
        harness.receive(stamp)
    first = harness.sample()
    second = harness.sample()

    assert first == second
    assert len(harness.fit_calls) == 3
    assert [entry[2] for entry in harness.tf_calls] == [
        10_000_000_000, 11_000_000_000, 12_000_000_000,
        10_000_000_000, 11_000_000_000, 12_000_000_000,
    ]

    # A different buffered image is never mistaken for a fit of the same stamp.
    harness.receive(12)
    harness.sample()
    assert len(harness.fit_calls) == 4


@pytest.mark.parametrize("change", ["tf", "intrinsics", "projection", "frame", "attachment"])
def test_changed_geometry_invalidates_fit(harness, change):
    harness.receive()
    harness.sample()
    if change == "tf":
        harness.pose.position.x -= 0.01
    elif change == "attachment":
        harness.attachment.attachment_revision += 1
        harness.source._receive_attachment_state(harness.attachment)
    else:
        if change == "intrinsics":
            harness.camera.k[0] += 1.0
        elif change == "projection":
            harness.camera.p[0] += 1.0
        else:
            harness.camera.header.frame_id = "other_camera"
        harness.source._receive_hand_depth_camera_info(harness.camera)
    harness.sample()
    assert len(harness.fit_calls) == 2
    assert len(harness.source._surface_distance_cache) == 1


def test_camera_message_timestamp_does_not_invalidate_calibration(harness):
    harness.receive()
    harness.sample()
    harness.camera.header.stamp.sec += 1
    harness.source._receive_hand_depth_camera_info(harness.camera)
    harness.sample()
    assert len(harness.fit_calls) == 1


def test_different_sensor_does_not_reuse_fit(harness):
    harness.receive()
    harness.sample()
    harness.source.surface_distance_samples("another_probe")
    assert len(harness.fit_calls) == 2
    assert harness.tf_calls[0][0] != harness.tf_calls[1][0]


@pytest.mark.parametrize("failure", ["tf_error", "fit_error"])
def test_failed_measurement_is_retried(harness, failure):
    harness.receive()
    setattr(harness, failure, "temporarily unavailable")
    with pytest.raises(ValueError, match="temporarily unavailable"):
        harness.sample()
    assert not harness.source._surface_distance_cache
    setattr(harness, failure, None)
    assert len(harness.sample()) == 1


def test_cached_fit_cannot_bypass_tf_failure(harness):
    harness.receive()
    harness.sample()
    harness.tf_error = "timestamped TF unavailable"
    with pytest.raises(ValueError, match="timestamped TF unavailable"):
        harness.sample()
    assert len(harness.fit_calls) == 1


def test_cached_fit_cannot_bypass_age_or_operation_start(harness):
    harness.receive()
    harness.sample()
    with pytest.raises(ValueError, match="post_start=0"):
        harness.sample(receipt_not_before=100.1)
    harness.now = 100.6
    with pytest.raises(ValueError, match="within_age=0"):
        harness.sample(maximum_age_sec=0.5)
    assert len(harness.fit_calls) == 1


def test_latest_valid_frame_must_remain_fresh_after_processing(harness):
    harness.receive()
    harness.fit_delay = 0.6
    with pytest.raises(ValueError, match="Latest valid.*stale"):
        harness.sample(maximum_age_sec=6.0, maximum_latest_age_sec=0.5)
    assert len(harness.source._surface_distance_cache) == 1

    # A subsequent poll can reuse the slow fit while adding a newer frame.
    harness.receive(11)
    harness.fit_delay = 0.0
    assert len(harness.sample(maximum_age_sec=6.0, maximum_latest_age_sec=0.5)) == 2
    assert len(harness.fit_calls) == 2


def test_recent_rejected_frame_does_not_refresh_valid_measurement(harness):
    harness.receive()
    harness.sample()
    harness.now = 100.6
    harness.receive(11)
    harness.fit_error = "no plane support"
    with pytest.raises(ValueError, match="Latest valid.*stale") as failure:
        harness.sample(maximum_age_sec=6.0, maximum_latest_age_sec=0.5)
    detail = str(failure.value)
    assert "newest raw frame age=0.000s" in detail
    assert "valid=1, rejected=1" in detail
    assert "last rejection: no plane support" in detail


def budgeted_sample(harness, **options):
    return harness.source._distance_samples(
        measurement_context=("hall_probe",), pose_target_frame="hall_probe_probe",
        measure=probe_surface_source.measure_probe_surface_distance,
        cache=harness.source._sensor_tip_distance_cache,
        **{"receipt_not_before": 0.0, "maximum_age_sec": 6.0, "minimum_samples": 1,
           "maximum_latest_age_sec": 0.5, "maximum_new_fits": 1, **options},
    )


def test_new_target_frame_preempts_failed_full_backlog(harness, monkeypatch):
    first_stamp = 10
    new_stamp = first_stamp + HAND_DEPTH_HISTORY_MAX_SAMPLES
    measure = probe_surface_source.measure_probe_surface_distance

    def reject_backlog(image, *args):
        harness.fit_error = None if image.header.stamp.sec == new_stamp else "no target support"
        return measure(image, *args)

    monkeypatch.setattr(probe_surface_source, "measure_probe_surface_distance", reject_backlog)
    for stamp in range(first_stamp, new_stamp):
        harness.receive(stamp)
        harness.now += 0.1
    with pytest.raises(ValueError, match="no target support"):
        budgeted_sample(harness)
    assert len(harness.fit_calls) == 1
    assert harness.fit_calls[-1][0].header.stamp.sec == new_stamp - 1

    harness.now += 0.05
    harness.receive(new_stamp)
    samples = budgeted_sample(harness)

    assert len(harness.fit_calls) == 2  # One fit per poll, including failures.
    assert harness.fit_calls[-1][0].header.stamp.sec == new_stamp
    assert [sample.stamp_seconds for sample in samples] == [float(new_stamp)]


def test_unchanged_failure_backlog_reaches_older_target_without_alternating(harness, monkeypatch):
    measure = probe_surface_source.measure_probe_surface_distance

    def reject_recent(image, *args):
        harness.fit_error = None if image.header.stamp.sec == 10 else "no target support"
        return measure(image, *args)

    monkeypatch.setattr(probe_surface_source, "measure_probe_surface_distance", reject_recent)
    for stamp in range(10, 14):
        harness.receive(stamp)
    for _ in range(3):
        with pytest.raises(ValueError, match="no target support"):
            budgeted_sample(harness)
    assert len(budgeted_sample(harness)) == 1
    assert [entry[0].header.stamp.sec for entry in harness.fit_calls] == [13, 12, 11, 10]


def test_budgeted_fit_retries_one_failed_frame_when_no_other_frame_exists(harness):
    harness.receive()
    harness.fit_error = "temporarily unavailable"
    with pytest.raises(ValueError, match="temporarily unavailable"):
        budgeted_sample(harness)
    harness.fit_error = None
    assert len(budgeted_sample(harness)) == 1
    assert len(harness.fit_calls) == 2


def test_newest_tf_delay_does_not_consume_fit_budget(harness):
    harness.receive(10)
    harness.receive(11)
    lookup = harness.source._lookup_pose
    delayed = [True]

    def lookup_with_delay(*args, lookup_time):
        if delayed[0] and lookup_time.nanoseconds == 11_000_000_000:
            raise ValueError("newest TF has not arrived")
        return lookup(*args, lookup_time=lookup_time)

    harness.source._lookup_pose = lookup_with_delay
    assert [sample.stamp_seconds for sample in budgeted_sample(harness)] == [10.0]
    assert len(harness.fit_calls) == 1
    delayed[0] = False
    assert [sample.stamp_seconds for sample in budgeted_sample(harness)] == [10.0, 11.0]
    assert len(harness.fit_calls) == 2


@pytest.mark.parametrize("change", ["tf", "calibration"])
def test_changed_geometry_retries_previous_failed_frame_before_older_backlog(harness, change):
    harness.receive(10)
    harness.receive(11)
    harness.fit_error = "temporarily unavailable"
    with pytest.raises(ValueError, match="temporarily unavailable"):
        budgeted_sample(harness)
    harness.fit_error = None
    if change == "tf":
        harness.pose.position.x += 0.01
    else:
        harness.camera.k[0] += 1.0
        harness.source._receive_hand_depth_camera_info(harness.camera)
    assert [sample.stamp_seconds for sample in budgeted_sample(harness)] == [11.0]
    assert len(harness.fit_calls) == 2


def test_cache_is_bounded_to_buffered_depth_and_cleared_on_destroy(harness):
    for stamp in range(10, 10 + HAND_DEPTH_HISTORY_MAX_SAMPLES + 3):
        harness.receive(stamp)
        harness.sample()
        assert len(harness.source._surface_distance_cache) <= HAND_DEPTH_HISTORY_MAX_SAMPLES
    assert set(harness.source._surface_distance_cache) == {
        id(image) for _, image in harness.source._hand_depth_history
    }
    harness.source.destroy()
    assert not harness.source._surface_distance_cache


@pytest.mark.parametrize("maximum_latest_age_sec", [0.0, -1.0, float("inf"), float("nan")])
def test_latest_age_limit_must_be_positive_finite(harness, maximum_latest_age_sec):
    with pytest.raises(ValueError, match="Maximum latest hand-depth age must be positive"):
        harness.sample(maximum_latest_age_sec=maximum_latest_age_sec)
