"""Offline native-goal checks for yaw correction at an adjusted body height."""

from dataclasses import replace
import math

import pytest
from bosdyn.api import robot_command_pb2
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn_spot_api_msgs.conversions import convert

from fault_detector_spot.navigation.base_motion_planner import BaseMotionPlanner
from fault_detector_spot.navigation.walking_profile import (
    GAITS, WalkingProfile, WalkingProfiles,
)


def planner():
    return BaseMotionPlanner(
        object(),
        WalkingProfiles(precision=WalkingProfile(
            relative_speed_mps=0.025,
            tag_speed_mps=0.06,
            angular_speed_rad_s=0.075,
            gait="crawl",
        )),
    )


@pytest.mark.parametrize("height", [-0.2, 0.2])
def test_yaw_correction_keeps_measured_position_height_and_precision_limits(height):
    motion_planner = planner()
    measured_pose = (2.3, -0.4, math.radians(-40))
    desired_yaw = math.radians(25)

    plan = motion_planner.resolve_yaw_correction(measured_pose, desired_yaw, height)
    goal = motion_planner.build_goal(plan)
    native = robot_command_pb2.RobotCommand()
    convert(goal.command, native)
    mobility = native.synchronized_command.mobility_command
    params = spot_command_pb2.MobilityParams()
    assert mobility.params.Unpack(params)
    trajectory = mobility.se2_trajectory_request
    point = trajectory.trajectory.points[0].pose

    assert motion_planner.planar_target(plan) == pytest.approx((2.3, -0.4, desired_yaw))
    assert trajectory.se2_frame_name == "odom"
    assert (point.position.x, point.position.y, point.angle) == pytest.approx(
        (2.3, -0.4, desired_yaw)
    )
    assert params.body_control.base_offset_rt_footprint.points[0].pose.position.z == (
        pytest.approx(height)
    )
    assert params.vel_limit.max_vel.linear.x == pytest.approx(0.025)
    assert params.vel_limit.max_vel.linear.y == pytest.approx(0.025)
    assert params.vel_limit.max_vel.angular == pytest.approx(0.075)
    assert params.locomotion_hint == GAITS["crawl"]
    assert not mobility.HasField("stand_request")


@pytest.mark.parametrize("height", [-0.21, 0.21, float("nan"), float("inf")])
def test_goal_build_rejects_invalid_plan_height(height):
    motion_planner = planner()
    plan = motion_planner.resolve_yaw_correction((0, 0, 0), 0.1, 0.1)

    with pytest.raises(ValueError, match="Body height"):
        motion_planner.build_goal(replace(plan, body_height_m=height))


@pytest.mark.parametrize("pose,yaw", [
    ((0, 0), 0),
    ((float("nan"), 0, 0), 0),
    ((0, 0, float("inf")), 0),
    ((0, 0, 0), float("nan")),
])
def test_yaw_correction_rejects_invalid_geometry(pose, yaw):
    with pytest.raises(ValueError):
        planner().resolve_yaw_correction(pose, yaw, 0.1)
