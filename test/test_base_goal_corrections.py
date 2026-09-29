"""Exercise correction lifecycle without ROS nodes or robot motion."""

from copy import deepcopy
from types import SimpleNamespace

import pytest
from fault_detector_msgs.msg import TagElement
from geometry_msgs.msg import PoseStamped, TransformStamped

from fault_detector_spot.navigation.base_correction_policy import (
    BaseCorrectionConfig,
    BaseCorrectionPolicy,
)
from fault_detector_spot.navigation.base_movement_executor import (
    BaseMovementExecutor,
    BaseMovementOutcome,
)
from fault_detector_spot.navigation.posture_state_source import (
    PostureState,
)
from fault_detector_spot.shared.geometry.movement_geometry import (
    MovementGeometryUnavailable,
)
from test_base_movement_executor import (
    FakeGoalHandle,
    FakePostureStateSource,
    ManualClock,
    ManualFuture,
)


class FakeTagStateSource:
    def __init__(self, x=1.0, stamp_sec=100.0):
        self.set_observation(x, stamp_sec)

    def set_observation(self, x, stamp_sec):
        tag = TagElement()
        tag.id = 7
        tag.pose.header.frame_id = "odom"
        whole = int(stamp_sec)
        tag.pose.header.stamp.sec = whole
        tag.pose.header.stamp.nanosec = round(
            (float(stamp_sec) - whole) * 1e9
        )
        tag.pose.pose.position.x = float(x)
        tag.pose.pose.orientation.w = 1.0
        self.tag = tag

    def clear(self):
        self.tag = None

    def visible_snapshot(self):
        return {} if self.tag is None else {7: deepcopy(self.tag)}

    def visible_tag_after(self, tag_id, boundary):
        if self.tag is None:
            return None
        if int(tag_id) != 7:
            return None
        stamp = (
            float(self.tag.pose.header.stamp.sec)
            + float(self.tag.pose.header.stamp.nanosec) * 1e-9
        )
        if stamp <= float(boundary):
            return None
        return deepcopy(self.tag)


class Client:
    def __init__(self):
        self.goals = []
        self.results = []
        self.handles = []

    def wait_for_server(self, timeout_sec):
        return True

    def send_goal_async(self, goal):
        self.goals.append(goal)
        result = ManualFuture()
        handle = FakeGoalHandle(result)
        self.results.append(result)
        self.handles.append(handle)
        send = ManualFuture()
        send.set_result(handle)
        return send


def make_movement(
    kind="relative",
    correction_config=None,
    tag_observation_timeout_sec=5.0,
):
    clock, client = ManualClock(), Client()
    transform = TransformStamped()
    transform.transform.rotation.w = 1.0
    target = PoseStamped()
    target.header.frame_id = "odom"
    target.pose.orientation.w = 1.0
    target.pose.position.x = 1.0
    tag_state_source = FakeTagStateSource()
    executor = BaseMovementExecutor(
        tf_listener=SimpleNamespace(
            lookup_a_tform_b=lambda *a, **k: transform
        ),
        action_client=client,
        monotonic_clock=clock,
        ros_time_sec=lambda: 100 + clock.now,
        posture_state_source=FakePostureStateSource(
            PostureState.STANDING
        ),
        tag_state_source=tag_state_source,
        correction_policy=BaseCorrectionPolicy(
            correction_config or BaseCorrectionConfig()
        ),
        tag_observation_timeout_sec=tag_observation_timeout_sec,
    )
    resolutions = []

    def resolve(_):
        resolutions.append(1)
        return target

    if kind == "tag":
        class TagCommand:
            tag_id = 7
            walking_profile = "precision"

            def __init__(self):
                self.tag_pose = PoseStamped()

            def compute_goal_pose(self, _):
                resolutions.append(1)
                return deepcopy(self.tag_pose)

        command = TagCommand()
    else:
        command = SimpleNamespace(
            compute_goal_pose=resolve,
            walking_profile="precision",
        )
    getattr(executor, kind)(command)

    def poll_at(x, advance=0, fresh=True):
        clock.now += advance
        transform.transform.translation.x = float(x)
        if fresh:
            stamp = 100 + clock.now
            transform.header.stamp.sec = int(stamp)
            transform.header.stamp.nanosec = round(
                (stamp - int(stamp)) * 1e9
            )
        return executor.poll()

    def complete(x, success=True):
        executor.poll()
        client.results[-1].set_result(
            SimpleNamespace(
                result=SimpleNamespace(success=success)
            )
        )
        return poll_at(x)

    return executor, client, resolutions, poll_at, complete


def test_relative_retry_reuses_same_absolute_goal_and_profile():
    executor, client, resolutions, poll, complete = make_movement("relative")
    complete(0.8)
    assert poll(0.8, 5).outcome is BaseMovementOutcome.RUNNING
    assert len(client.goals) == 2
    assert client.goals[0] == client.goals[1]
    assert len(resolutions) == 1
    complete(1.0)
    assert poll(1.0, 0.6).outcome is BaseMovementOutcome.SUCCESS
    assert not executor.active


def test_tag_correction_replans_from_stable_post_settle_observation():
    executor, client, resolutions, poll, complete = make_movement("tag")

    complete(0.8)
    executor.tag_state_source.set_observation(1.1, 100.7)
    assert poll(0.8, 0.6).outcome is BaseMovementOutcome.RUNNING

    executor.tag_state_source.set_observation(1.1, 100.8)
    assert poll(0.8, 0.1).outcome is BaseMovementOutcome.RUNNING

    executor.tag_state_source.set_observation(1.1, 100.9)
    update = poll(0.8, 0.1)

    assert update.outcome is BaseMovementOutcome.RUNNING
    assert "fresh tag-relative target" in update.detail
    assert len(client.goals) == 2
    assert len(resolutions) == 2
    assert executor._movement_plan.target.pose.position.x == pytest.approx(1.1)


def test_two_correction_attempts_then_failure():
    executor, client, _, poll, complete = make_movement()
    for x in (0.5, 0.65):
        complete(x)
        assert poll(x, 5).outcome is BaseMovementOutcome.RUNNING
    complete(0.8)
    update = poll(0.8, 5)
    assert update.outcome is BaseMovementOutcome.MOTION_FAILED
    assert "attempt limit" in update.detail
    assert len(client.goals) == 3


def test_no_progress_stops_early():
    _, client, _, poll, complete = make_movement()
    complete(0.8)
    poll(0.8, 5)
    complete(0.8)
    update = poll(0.8, 5)
    assert update.outcome is BaseMovementOutcome.MOTION_FAILED
    assert "insufficient progress" in update.detail
    assert len(client.goals) == 2


def test_stale_pose_at_deadline_does_not_retry_previous_error():
    _, client, _, poll, complete = make_movement()
    complete(0.8)
    update = poll(0.8, 5, fresh=False)
    assert update.outcome is BaseMovementOutcome.MOTION_FAILED
    assert len(client.goals) == 1


def test_execution_failure_is_not_retried():
    _, client, _, _, complete = make_movement()
    update = complete(0.8, success=False)
    assert update.outcome is BaseMovementOutcome.MOTION_FAILED
    assert len(client.goals) == 1


def test_cancel_during_correction_prevents_further_attempts():
    executor, client, _, poll, complete = make_movement()
    complete(0.8)
    poll(0.8, 5)
    executor.poll()
    executor.cancel()
    assert client.handles[-1].cancel_calls == 1
    assert executor.active

    client.results[-1].set_result(
        SimpleNamespace(
            result=SimpleNamespace(success=False)
        )
    )

    assert not executor.active
    assert executor._movement_plan is None
    assert executor.correction_policy.attempts == 0
    executor.poll()
    assert len(client.goals) == 2


def test_zero_corrections_disables_retry():
    _, client, _, poll, complete = make_movement(
        correction_config=BaseCorrectionConfig(
            maximum_attempts=0
        )
    )
    complete(0.8)
    assert (
        poll(0.8, 5).outcome
        is BaseMovementOutcome.MOTION_FAILED
    )
    assert len(client.goals) == 1


def test_missing_post_settle_tag_times_out():
    executor, _, _, poll, complete = make_movement(
        "tag",
        tag_observation_timeout_sec=1.0,
    )

    complete(0.8)
    executor.tag_state_source.clear()

    waiting = poll(0.8, 0.6)
    assert waiting.outcome is BaseMovementOutcome.RUNNING
    assert "post-settle tag 7" in waiting.detail

    failed = poll(0.8, 1.0)
    assert (
        failed.outcome
        is BaseMovementOutcome.TAG_OBSERVATION_TIMEOUT
    )
    assert "timed out after 1.0 s" in failed.detail
    assert not executor.active


def test_unstable_post_settle_tag_times_out():
    executor, _, _, poll, complete = make_movement(
        "tag",
        tag_observation_timeout_sec=0.5,
    )

    complete(0.8)
    executor.tag_state_source.set_observation(1.1, 100.7)
    assert poll(0.8, 0.6).outcome is BaseMovementOutcome.RUNNING

    executor.tag_state_source.set_observation(1.2, 100.8)
    assert poll(0.8, 0.2).outcome is BaseMovementOutcome.RUNNING

    executor.tag_state_source.set_observation(1.1, 101.2)
    failed = poll(0.8, 0.4)

    assert (
        failed.outcome
        is BaseMovementOutcome.TAG_OBSERVATION_TIMEOUT
    )
    assert "stable post-settle tag 7" in failed.detail
    assert not executor.active


def test_transient_tag_replan_geometry_is_bounded():
    executor, _, _, poll, complete = make_movement(
        "tag",
        tag_observation_timeout_sec=1.0,
    )

    complete(0.8)
    executor.motion_planner.resolve_tag_observation = (
        lambda *_: (_ for _ in ()).throw(
            MovementGeometryUnavailable("Waiting for TF")
        )
    )

    for stamp, advance in (
        (100.7, 0.6),
        (100.8, 0.1),
        (100.9, 0.1),
    ):
        executor.tag_state_source.set_observation(1.1, stamp)
        update = poll(0.8, advance)

    assert update.outcome is BaseMovementOutcome.RUNNING
    assert "tag re-planning geometry" in update.detail

    failed = poll(0.8, 0.9)
    assert failed.outcome is BaseMovementOutcome.MOTION_FAILED
    assert "timed out after 1.0 s" in failed.detail
    assert not executor.active


def test_unexpected_tag_replan_failure_is_reported():
    executor, _, _, poll, complete = make_movement("tag")

    complete(0.8)
    executor.motion_planner.resolve_tag_observation = (
        lambda *_: (_ for _ in ()).throw(
            ValueError("bad tag transform")
        )
    )

    for stamp, advance in (
        (100.7, 0.6),
        (100.8, 0.1),
        (100.9, 0.1),
    ):
        executor.tag_state_source.set_observation(1.1, stamp)
        update = poll(0.8, advance)

    assert update.outcome is BaseMovementOutcome.MOTION_FAILED
    assert "bad tag transform" in update.detail
    assert not executor.active
