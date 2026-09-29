"""Check configured walking profiles in the actual RobotCommand payload."""

from types import SimpleNamespace

import pytest
from bosdyn.api import robot_command_pb2
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn_spot_api_msgs.conversions import convert
from geometry_msgs.msg import PoseStamped

from fault_detector_spot.navigation.base_movement_executor import BaseMovementExecutor
from fault_detector_spot.navigation.walking_profile import WalkingProfile, WalkingProfiles


@pytest.mark.parametrize("name,tag_relative,speed,angular,gait", [
    ("normal", False, 0.10, 0.20, spot_command_pb2.HINT_AUTO),
    ("normal", True, 0.15, 0.20, spot_command_pb2.HINT_AUTO),
    ("precision", False, 0.05, 0.10, spot_command_pb2.HINT_SPEED_SELECT_CRAWL),
    ("precision", True, 0.05, 0.10, spot_command_pb2.HINT_SPEED_SELECT_CRAWL),
])
def test_profile_reaches_native_mobility_command(name, tag_relative, speed, angular, gait):
    target = PoseStamped()
    target.header.frame_id = "odom"
    target.pose.orientation.w = 1.0
    target.pose.position.x = 0.4
    profiles = WalkingProfiles(
        relative_profile=name if not tag_relative else "normal",
        tag_profile=name if tag_relative else "normal",
    )
    executor = BaseMovementExecutor(
        tf_listener=object(), walking_profiles=profiles,
        tag_state_source=SimpleNamespace(visible_snapshot=lambda: {7: SimpleNamespace(pose=target)}),
    )
    executor._prepare_move_command = lambda command, frame: command
    command = SimpleNamespace(tag_id=7, compute_goal_pose=lambda _: target)
    goal = executor._build_tag_goal(command) if tag_relative else executor._build_relative_goal(command)
    native = robot_command_pb2.RobotCommand()
    convert(goal.command, native)
    mobility = native.synchronized_command.mobility_command
    params = spot_command_pb2.MobilityParams()
    assert mobility.params.Unpack(params)
    assert params.locomotion_hint == gait
    assert params.vel_limit.max_vel.linear.x == pytest.approx(speed)
    assert params.vel_limit.max_vel.linear.y == pytest.approx(speed)
    assert params.vel_limit.min_vel.linear.x == pytest.approx(-speed)
    assert params.vel_limit.min_vel.linear.y == pytest.approx(-speed)
    assert params.vel_limit.max_vel.angular == pytest.approx(angular)
    assert params.vel_limit.min_vel.angular == pytest.approx(-angular)
    assert mobility.se2_trajectory_request.trajectory.points[0].pose.position.x == pytest.approx(0.4)
    assert not params.obstacle_params.disable_vision_body_obstacle_avoidance


@pytest.mark.parametrize("speed", [0, -1, float('nan'), float('inf')])
def test_invalid_speeds_fail_configuration(speed):
    with pytest.raises(ValueError):
        WalkingProfile(relative_speed_mps=speed)


def test_unknown_profile_and_gait_fail_configuration():
    with pytest.raises(ValueError):
        WalkingProfiles(tag_profile="typo")
    with pytest.raises(ValueError):
        WalkingProfile(gait="typo")


def test_ros_parameters_select_and_tune_precision():
    class Node:
        def __init__(self):
            self.values = {
                "base.walking.tag_profile": "precision",
                "base.walking.precision.tag_speed_mps": 0.04,
            }

        def has_parameter(self, name):
            return name in self.values

        def declare_parameter(self, name, value):
            self.values[name] = value

        def get_parameter(self, name):
            return SimpleNamespace(value=self.values[name])

    profiles = WalkingProfiles.from_node(Node())
    assert profiles.for_move().relative_speed_mps == 0.10
    assert profiles.for_move(True).tag_speed_mps == 0.04
    assert profiles.for_move(True).gait == "speed_select_crawl"
