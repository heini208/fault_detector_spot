"""Commanded targets retain readiness and force-guard ownership.

Only the guard owner is doubled here; its force sampling, stopping, and retreat
behavior are exercised by the dedicated guard tests. Executor construction and
the public command, readiness, and polling paths all run unchanged.
"""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from geometry_msgs.msg import PoseStamped

from fault_detector_spot.manipulation.arm_motion_speed import ArmMotionSpeed
from fault_detector_spot.manipulation.arm_movement_executor import ArmMovementExecutor
from fault_detector_spot.manipulation.arm_movement_result import (
    ArmMovementOutcome,
    ArmMovementUpdate,
)
from fault_detector_spot.manipulation.arm_state_source import ArmStowState
from fault_detector_spot.manipulation.guarded_probe_execution import GuardedProbeExecution
from fault_detector_spot.manipulation.probe_motion_planner import ResolvedProbeTarget


ENTRIES = (
    "relative",
    "absolute",
    "pose",
    "probe_pose",
    "tag_probe",
    "probe_relative",
    "orient_to_surface",
    "orient_to_tag",
    "restore_probe_checkpoint",
    "guarded_probe",
    "relative_precision",
    "absolute_precision",
    "tag_probe_precision",
)


def _target():
    pose = PoseStamped()
    pose.header.frame_id = "odom"
    pose.pose.position.x = 0.25
    pose.pose.orientation.w = 1.0
    return pose


def _invoke_entry(executor, entry):
    target = _target()
    options = dict(speed=ArmMotionSpeed(0.04, 0.2), force_threshold_n=1.5)
    method = entry.removesuffix("_precision")
    if method in {"relative", "absolute", "tag_probe"}:
        command = SimpleNamespace(
            offset=target,
            arm_speed_scale=1.0,
            precision_mode=entry.endswith("_precision"),
            tag_position_tolerance_m=0.01,
            precision_orientation_tolerance_rad=0.05,
            ignore_environment_collisions=False,
        )
        return getattr(executor, method)(command, **options)
    if method == "pose":
        return executor.pose(target, "odom", **options)
    if method in {"probe_pose", "probe_relative", "guarded_probe"}:
        return getattr(executor, method)(target, "sensor", **options)
    if method == "orient_to_surface":
        return executor.orient_to_surface("sensor", **options)
    if method == "orient_to_tag":
        return executor.orient_to_tag(7, "sensor", **options)
    if method == "restore_probe_checkpoint":
        return executor.restore_probe_checkpoint(ResolvedProbeTarget(target, "sensor"))
    raise AssertionError(f"Uncovered command entry: {entry}")


def _executor():
    source = SimpleNamespace(
        node=None,
        stow_state=Mock(return_value=ArmStowState.DEPLOYED),
    )
    action_client = Mock(spec=["server_is_ready", "send_goal_async"])
    executor = ArmMovementExecutor(
        object(),
        action_client=action_client,
        arm_state_source=source,
        tag_state_source=object(),
        surface_source=object(),
        force_contact_policy=object(),
        monotonic_clock=lambda: 10.0,
    )
    guard = Mock(spec=GuardedProbeExecution)
    guard.start.return_value = ArmMovementUpdate(
        ArmMovementOutcome.RUNNING, "Guard owns movement",
    )
    guard.poll.return_value = ArmMovementUpdate(
        ArmMovementOutcome.FORCE_STALE, "Force feedback became stale",
    )
    executor.guarded_probe_execution = guard
    return executor, source, action_client, guard


@pytest.mark.parametrize("entry", ENTRIES)
def test_command_waits_for_readiness_and_propagates_guard_failure(entry):
    executor, source, action_client, guard = _executor()
    source.stow_state.side_effect = [ArmStowState.UNKNOWN, ArmStowState.DEPLOYED]

    waiting = _invoke_entry(executor, entry)

    assert waiting.outcome is ArmMovementOutcome.RUNNING
    assert executor.active
    guard.start.assert_not_called()
    action_client.send_goal_async.assert_not_called()

    running = executor.poll()

    assert running is guard.start.return_value
    assert executor.active
    guard.start.assert_called_once()
    assert callable(guard.start.call_args.args[0])
    expected_threshold = None if entry == "restore_probe_checkpoint" else 1.5
    assert guard.start.call_args.kwargs == {"force_threshold_n": expected_threshold}
    action_client.send_goal_async.assert_not_called()

    failed = executor.poll()

    guard.poll.assert_called_once_with()
    assert failed.outcome is ArmMovementOutcome.FORCE_STALE
    assert failed.detail == "Force feedback became stale"
    assert not executor.active
    action_client.send_goal_async.assert_not_called()


@pytest.mark.parametrize("entry", ENTRIES)
@pytest.mark.parametrize("missing", ["guard", "force_policy"])
def test_command_cannot_bypass_missing_force_protection(entry, missing):
    executor, source, action_client, guard = _executor()
    if missing == "guard":
        executor.guarded_probe_execution = None
        expected_detail = "Guarded probe movement is not configured"
    else:
        executor.force_contact_policy = None
        expected_detail = "Guarded probe movement requires a force contact policy"

    result = _invoke_entry(executor, entry)

    assert result.outcome is ArmMovementOutcome.EXECUTION_ERROR
    assert result.detail == expected_detail
    assert not executor.active
    source.stow_state.assert_not_called()
    guard.start.assert_not_called()
    action_client.send_goal_async.assert_not_called()
