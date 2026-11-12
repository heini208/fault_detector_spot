"""Regression coverage for map extension admission and tracking-loss suspension."""
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


def test_initial_pose_and_process_liveness_are_not_mapping_permission():
    tracker, process, now, steady, sample = harness()
    assert not tracker.can_extend("plant", process)[0]
    sample(11)
    sample(12)
    assert not tracker.can_extend("plant", process)[0]
    sample(13)
    assert tracker.can_extend("plant", process)[0]
    assert not tracker.can_extend("another_map", process)[0]
    assert not tracker.can_extend("plant", object())[0]


def test_seed_and_loss_of_accepted_registrations_revoke_permission():
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
    assert tracker.snapshot()[0] == LocalizationState.LOST
    assert not tracker.can_extend("plant", process)[0]


def test_mapping_lost_tracking_requires_pause_and_latches_loss():
    tracker, process, now, steady, sample = harness()
    for second in (11, 12, 13):
        sample(second)
    tracker.begin_mapping("plant", process, existing_map=True)
    steady[0] = 17.0
    assert tracker.requires_pause(process)
    assert tracker.snapshot()[0] == LocalizationState.LOST
    tracker.pause_confirmed(process)
    assert tracker.snapshot()[2]
    assert not tracker.requires_pause(process)


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


def test_runtime_enters_mapping_after_three_accepted_matching_scans(runtime, tmp_path):
    manager, launches, stops = runtime
    (tmp_path / "plant.db").touch()
    process = manager.start_localization("plant")
    for stamp in (11, 12, 13):
        manager.node.get_clock.return_value.now.return_value.nanoseconds = (
            stamp * 1_000_000_000
        )
        manager._safety.tracker.observe_pose(pose(stamp))
        manager._safety.tracker.observe_info(registration(stamp))
    assert manager.start_mapping("plant") is process
    assert manager.is_mapping_running()
    assert not manager.nav2_runtime.is_running()
    assert len(launches) == 2
    manager._call_service.assert_called_with("/rtabmap/set_mode_mapping")


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


def test_rejected_lidar_registration_never_confirms_alignment():
    tracker, process, now, steady, sample = harness()
    for second in (11, 12, 13, 14):
        sample(second, accepted=False)
    assert tracker.snapshot()[0] == LocalizationState.LOCALIZING
    assert not tracker.can_extend("plant", process)[0]


def test_seed_during_mapping_requires_immediate_pause():
    tracker, process, now, steady, sample = harness()
    for second in (11, 12, 13):
        sample(second)
    tracker.begin_mapping("plant", process, existing_map=True)
    tracker.observe_seed(pose(13))
    assert tracker.snapshot()[0] == LocalizationState.LOST
    assert tracker.requires_pause(process)


def test_lost_alignment_during_mode_transition_is_latched():
    tracker, process, now, steady, sample = harness()
    for second in (11, 12, 13):
        sample(second)
    tracker.observe_seed(pose(13))
    tracker.begin_mapping("plant", process, existing_map=True)
    assert tracker.snapshot()[0] == LocalizationState.LOST
    assert tracker.requires_pause(process)


def test_watchdog_requests_pause_after_tracking_loss(runtime):
    manager, launches, stops = runtime
    manager.start_mapping("new_map")
    tracker = manager._safety.tracker
    tracker._mapping_since -= 10.0
    monitor = manager._safety
    monitor.tick()
    assert monitor._pause_future is not None
    monitor._pause_future.result(timeout=2)
    manager._call_service.assert_any_call("/rtabmap/pause", timeout_sec=3.0)
    assert tracker.snapshot()[2]
