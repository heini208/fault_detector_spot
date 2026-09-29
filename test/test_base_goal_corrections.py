"""Exercise correction lifecycle without ROS nodes or robot motion."""

from types import SimpleNamespace

import pytest
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
from test_base_movement_executor import (
    FakeGoalHandle,
    FakePostureStateSource,
    ManualClock,
    ManualFuture,
)


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


def make_movement(kind="relative", correction_config=None):
    clock, client = ManualClock(), Client()
    transform = TransformStamped()
    transform.transform.rotation.w = 1.0
    target = PoseStamped()
    target.header.frame_id = "odom"
    target.pose.orientation.w = 1.0
    target.pose.position.x = 1.0
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
        tag_state_source=SimpleNamespace(
            visible_snapshot=lambda: {
                7: SimpleNamespace(pose=target)
            }
        ),
        correction_policy=BaseCorrectionPolicy(
            correction_config or BaseCorrectionConfig()
        ),
    )
    resolutions = []

    def resolve(_):
        resolutions.append(1)
        return target

    command = SimpleNamespace(
        tag_id=7,
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


@pytest.mark.parametrize("kind", ["relative", "tag"])
def test_retries_same_absolute_goal_and_profile_then_succeeds(kind):
    executor, client, resolutions, poll, complete = make_movement(kind)
    complete(0.8)
    assert poll(0.8, 5).outcome is BaseMovementOutcome.RUNNING
    assert len(client.goals) == 2
    assert client.goals[0] == client.goals[1]
    assert len(resolutions) == 1
    complete(1.0)
    assert poll(1.0, 0.6).outcome is BaseMovementOutcome.SUCCESS
    assert not executor.active


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
