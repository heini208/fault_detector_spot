"""Expand real launch parameters while blocking all process execution."""

import importlib.util
from functools import partial
from pathlib import Path
from tempfile import NamedTemporaryFile
from types import SimpleNamespace
import xml.etree.ElementTree as ET

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    PopEnvironment,
    PopLaunchConfigurations,
    PushEnvironment,
    PushLaunchConfigurations,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.utilities import perform_substitutions
from launch_ros.actions import Node, SetParameter, SetParametersFromFile
import launch_ros.actions.node as node_actions
import pytest
import yaml


ROOT = Path(__file__).parents[1]
SENSORS = ROOT / "config/moveit_sensors.yaml"


def expand_configuration(entity, context):
    if isinstance(entity, LaunchDescription):
        children = entity.entities
    else:
        assert isinstance(entity, (
            GroupAction, IncludeLaunchDescription, Node, SetParameter,
            SetParametersFromFile, PushEnvironment, PopEnvironment,
            PushLaunchConfigurations, PopLaunchConfigurations,
        )), f"Unexpected launch action: {type(entity)}"
        children = entity.execute(context)
    for child in children or []:
        expand_configuration(child, context)


def flatten_parameters(parameters, prefix=""):
    result = {}
    for name, value in parameters.items():
        name = prefix + name
        if isinstance(value, dict):
            result.update(flatten_parameters(value, name + "."))
        else:
            result[name] = value
    return result


@pytest.fixture
def launch_parameters(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path))
    monkeypatch.setattr(
        node_actions, "NamedTemporaryFile", partial(NamedTemporaryFile, dir=tmp_path),
    )
    installed = Path(get_package_share_directory("spot_moveit_config"))
    original = tmp_path / "spot_moveit_config"
    (original / "config").mkdir(parents=True)
    (original / "launch").symlink_to(installed / "launch", target_is_directory=True)
    for path in (installed / "config").iterdir():
        if path.name != "spot.srdf":
            (original / "config" / path.name).symlink_to(path)
    semantic = ET.parse(installed / "config/spot.srdf")
    for joint in semantic.getroot().findall("virtual_joint"):
        semantic.getroot().remove(joint)
    ET.SubElement(semantic.getroot(), "virtual_joint", {
        "name": "odom_joint", "type": "floating",
        "parent_frame": "odom", "child_link": "body",
    })
    semantic.write(original / "config/spot.srdf")
    missing_packages = set()

    def package_share(package):
        if package in missing_packages:
            raise PackageNotFoundError(package)
        if package == "fault_detector_spot":
            return str(ROOT)
        if package == "spot_moveit_config":
            return str(original)
        if package == "moveit_ros_perception":
            return str(tmp_path / "moveit_ros_perception")
        return get_package_share_directory(package)

    monkeypatch.setattr(
        "ament_index_python.packages.get_package_share_directory", package_share,
    )
    spec = importlib.util.spec_from_file_location(
        "fault_detector_launch", ROOT / "launch/fault_detector_launch.py",
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    warnings = []
    monkeypatch.setattr(
        module, "get_logger", lambda _name: SimpleNamespace(warning=warnings.append),
    )
    captured = []

    def capture_process(node, context):
        command = [perform_substitutions(context, item) for item in node.cmd]
        paths = [
            Path(command[index + 1])
            for index, argument in enumerate(command) if argument == "--params-file"
        ]
        # launch_ros emits Python tuple tags for sequence parameters. These
        # files are generated locally from the installed, trusted launch.
        documents = [yaml.full_load(path.read_text()) for path in paths]
        captured.append((command, paths, documents))
        return None

    # Node.execute can prepare its ROS arguments, but never reaches a process
    # action or creates a ROS node, executor, publisher, or subscription.
    monkeypatch.setattr(ExecuteProcess, "execute", capture_process)
    return module, original, captured, missing_packages, warnings


@pytest.mark.parametrize("use_sim_time", ["false", "true"])
def test_sensors_are_scoped_to_original_move_group_without_changing_its_config(
    launch_parameters, use_sim_time,
):
    module, original, captured, _, warnings = launch_parameters
    baseline = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        str(original / "launch/move_group.launch.py"),
    ))
    expand_configuration(baseline, LaunchContext())
    assert len(captured) == 1
    baseline_parameters = next(iter(captured[0][2][0].values()))["ros__parameters"]

    description = module.generate_launch_description()
    assert warnings == []
    context = LaunchContext()
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    context.launch_configurations["use_sim_time"] = use_sim_time
    original_context = dict(context.launch_configurations)
    group, = [entity for entity in description.entities if isinstance(entity, GroupAction)]
    expand_configuration(group, context)
    assert len(captured) == 2
    command, paths, documents = captured[1]
    assert "__node:=move_group" in command
    assert len(paths) == 2 and paths[0] == SENSORS
    assert f"use_sim_time:={use_sim_time.title()}" in command
    assert next(iter(documents[1].values()))["ros__parameters"] == baseline_parameters
    assert {
        "robot_description", "robot_description_semantic",
        "robot_description_kinematics.arm.kinematics_solver",
        "default_planning_pipeline", "ompl.planning_plugin",
    } <= baseline_parameters.keys()
    sensor_parameters = documents[0]["/move_group"]["ros__parameters"]
    assert not baseline_parameters.keys() & flatten_parameters(sensor_parameters).keys()
    assert "use_sim_time" not in baseline_parameters
    assert context.launch_configurations == original_context

    sibling = Node(
        package="moveit_ros_move_group", executable="move_group", name="unrelated",
        parameters=[{"own_parameter": 1}],
    )
    expand_configuration(sibling, context)
    command, paths, documents = captured[2]
    assert SENSORS not in paths
    assert not any("use_sim_time:=" in argument for argument in command)
    assert next(iter(documents[0].values()))["ros__parameters"] == {"own_parameter": 1}


@pytest.mark.parametrize("missing", ["plugin", "joint", "type", "parent_frame", "child_link"])
def test_missing_prerequisite_warns_and_keeps_original_arm_planning(launch_parameters, missing):
    module, original, captured, missing_packages, warnings = launch_parameters
    if missing == "plugin":
        missing_packages.add("moveit_ros_perception")
    else:
        path = original / "config/spot.srdf"
        semantic = ET.parse(path)
        joint = semantic.getroot().find("virtual_joint")
        if missing == "joint":
            semantic.getroot().remove(joint)
        else:
            invalid = {"type": "fixed", "parent_frame": "map", "child_link": "hand"}
            joint.set(missing, invalid[missing])
        semantic.write(path)
    description = module.generate_launch_description()
    assert len(warnings) == 1
    assert "Native lidar occupancy unavailable:" in warnings[0]
    reason = "moveit_ros_perception" if missing == "plugin" else "floating odom -> body"
    assert reason in warnings[0]
    assert "arm planning still starts" in warnings[0]
    assert "collision toggle does not certify obstacle coverage" in warnings[0]
    context = LaunchContext()
    context.launch_configurations["use_sim_time"] = "true"
    group, = [entity for entity in description.entities if isinstance(entity, GroupAction)]
    expand_configuration(group, context)
    assert len(captured) == 1
    command, paths, documents = captured[0]
    assert "__node:=move_group" in command
    assert "use_sim_time:=True" in command
    assert len(paths) == 1 and SENSORS not in paths
    parameters = next(iter(documents[0].values()))["ros__parameters"]
    assert "robot_description" in parameters and "robot_description_semantic" in parameters
    assert "sensors" not in parameters and "octomap_frame" not in parameters
    assert "global_params" not in context.launch_configurations


def test_sensor_config_uses_corrected_lidar_and_native_updater():
    document = yaml.safe_load(SENSORS.read_text())
    assert list(document) == ["/move_group"]
    parameters = document["/move_group"]["ros__parameters"]
    assert parameters["octomap_frame"] == "odom"
    assert parameters["octomap_resolution"] == pytest.approx(0.05)
    assert parameters["sensors"] == ["lidar"]
    lidar = parameters["lidar"]
    assert lidar["sensor_plugin"] == "occupancy_map_monitor/PointCloudOctomapUpdater"
    assert lidar["point_cloud_topic"] == "/velodyne/points_sensor"
    assert lidar["max_range"] == 3.0
    assert lidar["point_subsample"] == 1
    assert lidar["padding_offset"] == 0.10
    assert lidar["padding_scale"] == 1.0
    assert lidar["max_update_rate"] == 0.0  # Adapter owns the replay-safe rate cap.
    assert lidar["filtered_cloud_topic"] == "/fault_detector/moveit/filtered_lidar"
