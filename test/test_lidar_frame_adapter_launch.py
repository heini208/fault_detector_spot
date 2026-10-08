"""Inspect standalone lidar launch wiring without starting ROS nodes."""

import importlib.util
import math
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.events import Shutdown
from launch.utilities import perform_substitutions
import pytest
import yaml


ROOT = Path(__file__).parents[1]


@pytest.fixture
def launch_file(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path))
    spec = importlib.util.spec_from_file_location(
        "lidar_adapter_launch", ROOT / "launch/lidar_frame_adapter_launch.py",
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, "get_package_share_directory", lambda _name: str(ROOT))
    monkeypatch.setattr(module, "Node", lambda **kwargs: dict(kind="node", **kwargs))
    monkeypatch.setattr(module, "ComposableNode", lambda **kwargs: kwargs)
    monkeypatch.setattr(
        module, "ComposableNodeContainer", lambda **kwargs: dict(kind="container", **kwargs),
    )
    return module


def defaults(module):
    context = LaunchContext()
    description = module.generate_launch_description()
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            context.launch_configurations[entity.name] = perform_substitutions(
                context, entity.default_value,
            )
    return context, [entity for entity in description.entities if isinstance(entity, dict)]


def test_default_launch_only_starts_adapter_and_calibrated_mount(launch_file):
    context, nodes = defaults(launch_file)
    assert context.launch_configurations == {
        "input_topic": "/velodyne/points",
        "output_topic": "/velodyne/points_sensor",
        "sensor_frame": "lidar_sensor",
        "use_sim_time": "false",
        "publish_mount_tf": "true",
        "calibration_file": str(ROOT / "config/lidar_mount_calibration.yaml"),
    }
    assert [(node["package"], node["executable"]) for node in nodes] == [
        ("rclcpp_components", "component_container"),
        ("fault_detector_spot", "lidar_frame_adapter"),
    ]
    mount, adapter = nodes
    assert mount["condition"].evaluate(context)
    component, = mount["composable_node_descriptions"]
    assert component["package"] == "tf2_ros"
    assert component["plugin"] == "tf2_ros::StaticTransformBroadcasterNode"
    assert component["name"] == "lidar_sensor_mount"
    assert component["namespace"] == ""
    assert component["parameters"][0].perform(context) == (
        context.launch_configurations["calibration_file"]
    )
    assert component["parameters"][1]["child_frame_id"].perform(context) == "lidar_sensor"
    assert adapter["parameters"][0]["sensor_frame"].perform(context) == "lidar_sensor"
    assert [(source, target.perform(context)) for source, target in adapter["remappings"]] == [
        ("input", "/velodyne/points"), ("output", "/velodyne/points_sensor"),
    ]


def test_adapter_exit_shuts_down_the_standalone_mount_container(launch_file, monkeypatch):
    context, nodes = defaults(launch_file)
    _mount, adapter = nodes
    events = []
    monkeypatch.setattr(context, "emit_event_sync", events.append)

    for action in adapter["on_exit"]:
        action.execute(context)

    event, = events
    assert isinstance(event, Shutdown)


@pytest.mark.parametrize("publish_mount_tf", ["true", "false"])
def test_existing_tf_and_custom_topics_share_target_and_clock(launch_file, publish_mount_tf):
    context, nodes = defaults(launch_file)
    context.launch_configurations.update({
        "input_topic": "/robot/raw_points", "output_topic": "/robot/sensor_points",
        "sensor_frame": "robot/physical_lidar", "use_sim_time": "true",
        "publish_mount_tf": publish_mount_tf,
        "calibration_file": "/tmp/custom_lidar_mount.yaml",
    })
    mount, adapter = nodes
    assert mount["condition"].evaluate(context) is (publish_mount_tf == "true")
    component, = mount["composable_node_descriptions"]
    assert component["parameters"][0].perform(context) == "/tmp/custom_lidar_mount.yaml"
    assert component["parameters"][1]["child_frame_id"].perform(context) == "robot/physical_lidar"
    assert component["parameters"][1]["use_sim_time"].perform(context) == "true"
    assert adapter["parameters"][0]["sensor_frame"].perform(context) == "robot/physical_lidar"
    assert adapter["parameters"][0]["use_sim_time"].perform(context) == "true"
    assert [(source, target.perform(context)) for source, target in adapter["remappings"]] == [
        ("input", "/robot/raw_points"), ("output", "/robot/sensor_points"),
    ]


def test_default_calibration_preserves_previous_sdk_mount_transform():
    document = yaml.safe_load((ROOT / "config/lidar_mount_calibration.yaml").read_text())
    assert list(document) == ["/lidar_sensor_mount"]
    parameters = document["/lidar_sensor_mount"]["ros__parameters"]
    assert parameters == {
        "frame_id": "body",
        "translation.x": -0.18778000000000006,
        "translation.y": -4.440892098500626e-16,
        "translation.z": 0.14213000000000076,
        "rotation.x": -1.5699247457590104e-16,
        "rotation.y": 3.0964814046186007e-16,
        "rotation.z": 0.30901699437494723,
        "rotation.w": 0.9510565162951541,
    }
    norm = math.sqrt(sum(parameters[f"rotation.{axis}"] ** 2 for axis in "xyzw"))
    assert norm == pytest.approx(1.)
    assert "child_frame_id" not in parameters
