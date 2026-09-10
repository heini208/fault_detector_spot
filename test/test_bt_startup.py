"""Offline checks for command readiness and the reduced conversion registry."""

import subprocess
import sys

from fault_detector_spot.application.behaviour_tree.behaviours.command_subscriber import (
    CommandSubscriber,
)
from test_command_ros_transport import FakeNode


def test_deferred_consumer_is_advertised_only_after_activation():
    node = FakeNode()
    consumer = CommandSubscriber(defer_subscription=True)
    consumer.setup(node=node)
    assert consumer.request_topic not in node.subscriptions
    consumer.activate()
    subscription = consumer.request_subscription
    assert consumer.request_topic in node.subscriptions
    consumer.activate()
    assert consumer.request_subscription is subscription


def test_command_conversions_work_without_umbrella_registry():
    # A fresh interpreter prevents another test from registering missing types.
    subprocess.run(
        [sys.executable, "-c", """
import sys
from bosdyn.client.robot_command import RobotCommandBuilder as B
from bosdyn.api.robot_command_pb2 import RobotCommand as Proto
from bosdyn_spot_api_msgs.conversions import convert
from spot_msgs.action import RobotCommand
commands = [
    B.claw_gripper_open_fraction_command(1.0),
    B.claw_gripper_open_fraction_command(0.0),
    B.arm_stow_command(),
    B.arm_ready_command(),
    B.synchro_stand_command(),
    B.arm_pose_command(0.5, 0, 0.2, 1, 0, 0, 0, "body", 2),
    B.synchro_se2_trajectory_point_command(
        0.1, 0, 0, "odom", params=B.mobility_params()),
]
for command in commands:
    goal = RobotCommand.Goal()
    convert(command, goal.command)
    recovered = Proto()
    convert(goal.command, recovered)
    assert recovered == command
assert "bosdyn_msgs.conversions" not in sys.modules
assert "bosdyn_autowalk_api_msgs.conversions" not in sys.modules
assert "bosdyn_graph_nav_api_msgs.conversions" not in sys.modules
"""],
        check=True,
        timeout=30,
    )
