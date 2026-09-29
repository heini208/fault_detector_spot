"""Validate semantic tag re-planning in BaseMotionPlanner."""

from copy import deepcopy
from types import SimpleNamespace

from geometry_msgs.msg import PoseStamped

from fault_detector_spot.navigation.base_motion_planner import (
    BaseMotionPlanner,
)
from fault_detector_spot.navigation.walking_profile import WalkingProfiles


class SemanticTagCommand:
    tag_id = 7
    walking_profile = "precision"

    def __init__(self):
        self.tag_pose = PoseStamped()
        self.tag_pose.header.frame_id = "odom"
        self.tag_pose.pose.orientation.w = 1.0

    def compute_goal_pose(self, _):
        return deepcopy(self.tag_pose)


def observation(x):
    pose = PoseStamped()
    pose.header.frame_id = "odom"
    pose.pose.position.x = float(x)
    pose.pose.orientation.w = 1.0
    return SimpleNamespace(id=7, pose=pose)


def test_tag_replan_uses_new_observation_without_mutating_semantic_command():
    planner = BaseMotionPlanner(object(), WalkingProfiles())
    command = SemanticTagCommand()
    original_x = command.tag_pose.pose.position.x

    first = planner.resolve_tag_observation(
        command,
        observation(1.0),
    )
    second = planner.resolve_tag_observation(
        command,
        observation(1.2),
    )

    assert first.target.pose.position.x == 1.0
    assert second.target.pose.position.x == 1.2
    assert command.tag_pose.pose.position.x == original_x


def test_tag_replan_rejects_observation_for_another_tag():
    planner = BaseMotionPlanner(object(), WalkingProfiles())
    command = SemanticTagCommand()
    wrong = observation(1.0)
    wrong.id = 8

    try:
        planner.resolve_tag_observation(command, wrong)
    except ValueError:
        pass
    else:
        raise AssertionError("Mismatched tag observation was accepted")


class MovingTF:
    def __init__(self):
        self.yaw = 0.0
        self.x = 0.0
        self.requested_times = []
        self._tf_buffer = SimpleNamespace(can_transform=lambda *args: True)

    def lookup_a_tform_b(self, target, source, transform_time=None, **kwargs):
        import math
        from geometry_msgs.msg import TransformStamped
        transform = TransformStamped()
        transform.header.frame_id = target
        # Capture-time body pose was the origin; latest pose has moved.
        yaw = self.yaw if transform_time is None else 0.0
        transform.transform.translation.x = self.x if transform_time is None else 0.0
        transform.transform.rotation.z = math.sin(yaw / 2)
        transform.transform.rotation.w = math.cos(yaw / 2)
        self.requested_times.append(transform_time)
        return transform


def real_command(frame="body"):
    import math
    from builtin_interfaces.msg import Time
    from fault_detector_spot.navigation.commands.base_to_tag_command import BaseToTagCommand
    offset = PoseStamped()
    offset.header.frame_id = frame
    offset.pose.position.x = -0.5
    offset.pose.orientation.z = math.sin(math.radians(30) / 2)
    offset.pose.orientation.w = math.cos(math.radians(30) / 2)
    return BaseToTagCommand("test", Time(), observation(1.0).pose, 7, offset)


def test_body_offset_is_frozen_for_subsequent_tag_replans():
    import math
    import pytest
    listener = MovingTF()
    planner = BaseMotionPlanner(listener, WalkingProfiles())
    original = real_command()
    prepared = planner.prepare_tag_request(original)
    first = planner.resolve_tag_observation(prepared, observation(1.0))
    listener.yaw = math.radians(30)
    second = planner.resolve_tag_observation(prepared, observation(1.1))
    assert planner.planar_target(first) == pytest.approx((0.5, 0, math.radians(30)))
    assert planner.planar_target(second) == pytest.approx((0.6, 0, math.radians(30)))
    assert original.offset.header.frame_id == "body"


def test_capture_time_tf_is_used_for_body_observation():
    import pytest
    listener = MovingTF()
    listener.x = 0.1
    planner = BaseMotionPlanner(listener, WalkingProfiles())
    tag = observation(1.0)
    tag.pose.header.frame_id = "body"
    tag.pose.header.stamp.sec = 100
    plan = planner.resolve_tag_observation(real_command("odom"), tag)
    assert plan.target.pose.position.x == pytest.approx(0.5)
    assert listener.requested_times[0].nanoseconds == 100_000_000_000


def test_missing_historical_tf_does_not_fall_back_to_latest():
    import pytest
    from tf2_ros import ExtrapolationException
    from fault_detector_spot.shared.geometry.movement_geometry import MovementGeometryUnavailable
    listener = MovingTF()
    def unavailable(*args, **kwargs):
        raise ExtrapolationException("capture outside TF history")
    listener.lookup_a_tform_b = unavailable
    planner = BaseMotionPlanner(listener, WalkingProfiles())
    tag = observation(1.0)
    tag.pose.header.frame_id = "body"
    tag.pose.header.stamp.sec = 100
    with pytest.raises(MovementGeometryUnavailable):
        planner.resolve_tag_observation(real_command("odom"), tag)


def test_executor_retains_frozen_request_after_initial_submission():
    import math
    import pytest
    from test_base_goal_corrections import Client
    from test_base_movement_executor import FakePostureStateSource
    from fault_detector_spot.navigation.base_movement_executor import BaseMovementExecutor, BaseMovementOutcome
    from fault_detector_spot.navigation.posture_state_source import PostureState
    listener = MovingTF()
    executor = BaseMovementExecutor(
        listener,
        action_client=Client(),
        posture_state_source=FakePostureStateSource(PostureState.STANDING),
        tag_state_source=SimpleNamespace(visible_snapshot=lambda: {7: observation(1.0)}),
    )
    assert executor.tag(real_command()).outcome is BaseMovementOutcome.RUNNING
    listener.yaw = math.radians(30)
    corrected = executor.motion_planner.resolve_tag_observation(
        executor._semantic_tag_command, observation(1.1),
    )
    assert executor.motion_planner.planar_target(corrected) == pytest.approx(
        (0.6, 0, math.radians(30))
    )
