"""Regression coverage for map extension admission without automatic mapping suspension."""
import pytest
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseWithCovarianceStamped
from rtabmap_msgs.msg import Info

from fault_detector_spot.navigation.localization_state import (
    LocalizationState, LocalizationStateTracker,
)
from test_runtime_managers import runtime  # noqa: F401 - shared runtime fixture


def pose(stamp):
    message = PoseWithCovarianceStamped()
    message.header.frame_id = "map"
    message.header.stamp = Time(sec=int(stamp), nanosec=0)
    message.pose.pose.orientation.w = 1.0
    for index in (0, 7, 35):
        message.pose.covariance[index] = 0.01
    return message


def registration(stamp, accepted=True):
    message = Info()
    message.header.frame_id = "map"
    message.header.stamp = Time(sec=int(stamp), nanosec=0)
    message.proximity_detection_id = 1 if accepted else 0
    message.loop_closure_id = 0
    message.stats_keys = [
        "Loop/RejectedHypothesis/",
        "Proximity/Space_detections_added_icp_global/",
        "Proximity/Space_detections_added_icp_multi/",
    ]
    message.stats_values = [0.0, 0.0, 1.0 if accepted else 0.0]
    return message


def harness():
    now = [10]
    steady = [10.0]
    tracker = LocalizationStateTracker(
        lambda: now[0] * 1_000_000_000,
        steady_clock=lambda: steady[0],
        required_matches=3,
    )
    process = object()
    tracker.begin_localization("plant", process)

    def sample(stamp, accepted=True):
        now[0] = stamp
        steady[0] = float(stamp)
        tracker.observe_pose(pose(stamp))
        tracker.observe_info(registration(stamp, accepted=accepted))

    return tracker, process, now, steady, sample


def test_mapping_requires_matching_session_and_one_confident_pose():
    tracker, process, now, steady, sample = harness()
    assert not tracker.can_extend("plant", process)[0]
    now[0] = 11
    tracker.observe_pose(pose(11))
    assert tracker.can_extend("plant", process)[0]
    assert not tracker.can_extend("another_map", process)[0]
    assert not tracker.can_extend("plant", object())[0]


def test_seed_revokes_permission_but_pose_age_only_changes_diagnostics():
    tracker, process, now, steady, sample = harness()
    for second in (11, 12, 13):
        sample(second)
    assert tracker.can_extend("plant", process)[0]
    tracker.observe_seed(pose(13))
    assert not tracker.can_extend("plant", process)[0]
    for second in (14, 15, 16):
        sample(second)
    assert tracker.can_extend("plant", process)[0]
    steady[0] = 20.0
    now[0] = 20
    assert tracker.snapshot()[0] == LocalizationState.LOST
    assert tracker.can_extend("plant", process)[0]


def test_runtime_rejects_saved_map_without_localization(runtime, tmp_path):
    manager, launches, stops = runtime
    (tmp_path / "plant.db").touch()
    with pytest.raises(RuntimeError, match="Cannot extend existing map"):
        manager.start_mapping("plant")
    assert not launches
    manager.start_localization("plant")
    with pytest.raises(RuntimeError, match="Cannot extend existing map"):
        manager.start_mapping("plant")
    assert manager.is_localization_running()
    assert len(launches) == 2


@pytest.mark.parametrize("registration_result", ["missing", "rejected"])
def test_runtime_enters_mapping_with_confident_pose_without_registration(runtime, tmp_path, registration_result):
    manager, launches, stops = runtime
    (tmp_path / "plant.db").touch()
    process = manager.start_localization("plant")
    manager.node.get_clock.return_value.now.return_value.nanoseconds = 11_000_000_000
    manager._safety.tracker.observe_pose(pose(11))
    if registration_result == "rejected":
        manager._safety.tracker.observe_info(registration(11, accepted=False))
    assert manager.start_mapping("plant") is process
    assert manager.is_mapping_running()
    assert not manager.nav2_runtime.is_running()
    assert len(launches) == 2
    manager._call_service.assert_called_with("/rtabmap/set_mode_mapping")


@pytest.mark.parametrize("entrypoint", ["start_mapping", "set_mode_mapping"])
def test_stopped_localization_cannot_reuse_verified_evidence(runtime, tmp_path, entrypoint):
    manager, launches, stops = runtime
    database = tmp_path / "plant.db"
    database.write_bytes(b"saved map")
    process = manager.start_localization("plant")
    for stamp in (11, 12, 13):
        manager.node.get_clock.return_value.now.return_value.nanoseconds = (
            stamp * 1_000_000_000
        )
        manager._safety.tracker.observe_pose(pose(stamp))
        manager._safety.tracker.observe_info(registration(stamp))
    assert manager._safety.tracker.can_extend("plant", process)[0]

    # An unexpected exit retains the process reference and localization evidence.
    process.alive = False
    assert manager.get_running_mode() == manager.MODE_NONE
    manager._call_service.reset_mock()
    with pytest.raises(RuntimeError, match="start localization for this map"):
        getattr(manager, entrypoint)()

    manager._call_service.assert_not_called()
    assert manager.nav2_runtime.is_running()
    assert len(launches) == 2
    assert stops == []
    assert database.read_bytes() == b"saved map"


def test_existing_map_requires_localization_of_the_same_active_map(runtime, tmp_path):
    manager, launches, stops = runtime
    (tmp_path / "plant.db").touch()
    (tmp_path / "other.db").touch()
    process = manager.start_localization("other")
    manager._call_service.reset_mock()

    with pytest.raises(RuntimeError, match="start localization for this map"):
        manager.start_mapping("plant")

    manager._call_service.assert_not_called()
    assert manager.process is process
    assert manager.is_localization_running()
    assert len(launches) == 2
    assert stops == []


def test_repeated_start_mapping_preserves_the_active_saved_map(runtime, tmp_path):
    manager, launches, stops = runtime
    process = manager.start_mapping("plant")
    (tmp_path / "plant.db").touch()
    manager._call_service.reset_mock()

    assert manager.start_mapping("plant") is process
    manager.set_mode_mapping()

    manager._call_service.assert_not_called()
    assert manager.is_mapping_running()
    assert len(launches) == 1
    assert stops == []


def test_runtime_never_deletes_saved_database_through_start_mapping(runtime, tmp_path):
    manager, launches, stops = runtime
    (tmp_path / "plant.db").touch()
    with pytest.raises(RuntimeError, match="Refusing to overwrite"):
        manager.start_mapping("plant", delete_db=True)
    assert not launches


def test_new_map_remains_creatable_without_prior_localization(runtime):
    manager, launches, stops = runtime
    manager.start_mapping("brand_new")
    assert manager.is_mapping_running()
    assert "extend_map:=true" in launches[0]


def test_rejected_registration_diagnostic_does_not_block_confident_pose():
    tracker, process, now, steady, sample = harness()
    for second in (11, 12, 13, 14):
        sample(second, accepted=False)
    assert tracker.snapshot()[0] == LocalizationState.LOCALIZING
    assert tracker.can_extend("plant", process)[0]


def test_seed_during_mapping_invalidates_alignment():
    tracker, process, now, steady, sample = harness()
    for second in (11, 12, 13):
        sample(second)
    tracker.begin_mapping("plant", process, existing_map=True)
    tracker.observe_seed(pose(13))
    assert tracker.snapshot()[0] == LocalizationState.LOST
    assert not tracker.snapshot()[2]


def test_lost_alignment_during_mode_transition_is_latched():
    tracker, process, now, steady, sample = harness()
    for second in (11, 12, 13):
        sample(second)
    tracker.observe_seed(pose(13))
    tracker.begin_mapping("plant", process, existing_map=True)
    assert tracker.snapshot()[0] == LocalizationState.LOST
    assert not tracker.snapshot()[2]


def test_map_extension_accepts_confident_latest_pose_older_than_quality_limit():
    tracker, process, now, steady, sample = harness()
    for second in (11, 12, 13):
        sample(second)
    # Still within the three-second stream timeout, but beyond pose freshness.
    now[0] = 15
    steady[0] = 15.0
    assert tracker.can_extend("plant", process)[0]


@pytest.mark.parametrize("observation", ["missing", "stale", "invalid", "seed"])
def test_mapping_observations_never_pause_or_stop_runtime(runtime, observation):
    manager, launches, stops = runtime
    process = manager.start_mapping("new_map")
    tracker = manager._safety.tracker
    tracker._steady_clock = lambda: 1000.0
    tracker._ros_clock_ns = lambda: 1000_000_000_000
    if observation in ("stale", "invalid"):
        message = pose(11 if observation == "stale" else 1000)
        if observation == "invalid":
            message.pose.covariance[0] = float("nan")
        tracker.observe_pose(message)
        tracker.observe_info(registration(message.header.stamp.sec, accepted=False))
    elif observation == "seed":
        tracker.observe_seed(pose(1000))
    manager._call_service.reset_mock()
    manager._publish_runtime_status()
    manager._call_service.assert_not_called()
    assert manager.process is process
    assert manager.is_mapping_running()
    assert not stops
    assert not tracker.snapshot()[2]


@pytest.mark.parametrize("covariance", [0.0, 9999.0, float("nan"), 0.10])
def test_mapping_rejects_uncertain_or_invalid_pose_without_registration(covariance):
    tracker, process, now, steady, sample = harness()
    now[0] = 11
    message = pose(11)
    message.pose.covariance[0] = covariance
    tracker.observe_pose(message)
    assert not tracker.can_extend("plant", process)[0]


def test_latest_low_confidence_pose_replaces_previous_good_estimate():
    tracker, process, now, steady, sample = harness()
    sample(11)
    now[0] = 12
    message = pose(12)
    message.pose.covariance[0] = 1.0
    tracker.observe_pose(message)
    now[0] = 100
    assert not tracker.can_extend("plant", process)[0]


def test_nav2_shutdown_delay_does_not_expire_mapping_admission(runtime, tmp_path, monkeypatch):
    manager, _, _ = runtime
    (tmp_path / "plant.db").touch()
    process = manager.start_localization("plant")
    manager.node.get_clock.return_value.now.return_value.nanoseconds = 11_000_000_000
    manager._safety.tracker.observe_pose(pose(11))
    stop = manager.nav2_runtime.stop

    def delayed_stop():
        manager.node.get_clock.return_value.now.return_value.nanoseconds = 20_000_000_000
        return stop()

    monkeypatch.setattr(manager.nav2_runtime, "stop", delayed_stop)
    assert manager.start_mapping("plant") is process
    assert manager.is_mapping_running()


@pytest.mark.parametrize("failure", ["service", "seed", "confidence"])
def test_failed_mapping_transition_restores_nav2_and_rviz_toggle_preserves_it(
    runtime, tmp_path, monkeypatch, failure,
):
    manager, _, _ = runtime
    (tmp_path / "plant.db").touch()
    process = manager.start_localization("plant")
    manager.node.get_clock.return_value.now.return_value.nanoseconds = 11_000_000_000
    manager._safety.tracker.observe_pose(pose(11))
    stop = manager.nav2_runtime.stop

    def stop_with_changed_evidence():
        result = stop()
        if failure == "seed":
            manager._safety.tracker.observe_seed(pose(11))
        elif failure == "confidence":
            manager.node.get_clock.return_value.now.return_value.nanoseconds = 12_000_000_000
            message = pose(12)
            message.pose.covariance[0] = 1.0
            manager._safety.tracker.observe_pose(message)
        return result

    monkeypatch.setattr(manager.nav2_runtime, "stop", stop_with_changed_evidence)
    if failure == "service":
        manager._call_service.return_value = False
    with pytest.raises(RuntimeError, match="Could not switch|Cannot extend"):
        manager.start_mapping("plant")
    assert manager.process is process
    assert manager.is_localization_running()
    nav2 = manager.nav2_runtime.process
    manager.set_visualization(False)
    assert manager.nav2_runtime.process is nav2
    assert manager.is_localization_running()
