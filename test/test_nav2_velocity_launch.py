"""Expand Nav2 launch remapping without starting ROS processes or nodes."""

import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import LoadComposableNodes
import pytest

from fault_detector_spot.application.behaviour_tree.behaviours.robot_command_resources import RobotCommandResources


ROOT = Path(__file__).parents[1]


@pytest.mark.parametrize("application_owned", [False, True])
def test_nav2_final_outputs_use_selected_topic_without_bypassing_velocity_smoother(monkeypatch, tmp_path, application_owned):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path))
    monkeypatch.setattr(
        "ament_index_python.packages.get_package_share_directory",
        lambda package: str(ROOT) if package == "fault_detector_spot" else get_package_share_directory(package),
    )
    spec = importlib.util.spec_from_file_location("nav2_velocity_launch", ROOT / "launch/nav2_lidar_launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    captured = {}

    def capture_process(node, context):
        executable = Path(node.cmd[0][0].perform(context)).name
        captured[executable] = node.expanded_remapping_rules
        return None

    # Node prepares launch arguments only; every process execution is replaced.
    monkeypatch.setattr(ExecuteProcess, "execute", capture_process)

    def expand(entity, context):
        condition = getattr(entity, "condition", None)
        if condition is not None and not condition.evaluate(context):
            return
        assert not isinstance(entity, LoadComposableNodes), "This launch defaults to standalone nodes"
        children = entity.entities if isinstance(entity, LaunchDescription) else entity.execute(context)
        for child in children or []:
            expand(child, context)

    context = LaunchContext()
    if application_owned:
        context.launch_configurations["velocity_topic"] = RobotCommandResources.NAVIGATION_VELOCITY_TOPIC
    expand(module.generate_launch_description(), context)

    def destination(node, topic):
        # ROS uses the first applicable rule; node-qualified rules must precede
        # stock Nav2 rules without changing any other node's input remapping.
        return next(target for source, target in captured[node]
                    if source in {topic, f"{node}:{topic}"})

    private = RobotCommandResources.NAVIGATION_VELOCITY_TOPIC if application_owned else "cmd_vel"
    assert destination("controller_server", "cmd_vel") == "cmd_vel_nav"
    assert destination("velocity_smoother", "cmd_vel") == "cmd_vel_nav"
    assert destination("velocity_smoother", "cmd_vel_smoothed") == private
    assert destination("behavior_server", "cmd_vel") == private
