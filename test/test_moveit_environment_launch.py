"""Resolve environmental sensor selection without launching any ROS process."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import GroupAction
from launch_ros.actions import ComposableNodeContainer, SetParameter, SetParametersFromFile
from launch_ros.actions.load_composable_nodes import get_composable_node_load_request
from rclpy.parameter import Parameter
import pytest
import yaml


ROOT = Path(__file__).parents[1]


@pytest.mark.parametrize("environment", [False, True])
@pytest.mark.parametrize("lidar", [False, True])
def test_launch_enables_exactly_the_selected_sensor_updaters(monkeypatch, tmp_path, environment, lidar):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path))
    spec = importlib.util.spec_from_file_location(
        "environment_launch_under_test", ROOT / "launch/fault_detector_launch.py",
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(
        module, "get_package_share_directory",
        lambda name: str(ROOT if name == "fault_detector_spot" else ROOT.parent / name),
    )
    components = []

    def capture_container(**kwargs):
        components.extend(kwargs["composable_node_descriptions"])
        return ComposableNodeContainer(**kwargs)

    monkeypatch.setattr(module, "ComposableNodeContainer", capture_container)
    description = module.generate_launch_description()
    moveit_group, lidar_group = [entity for entity in description.entities if isinstance(entity, GroupAction)]
    context = LaunchContext()
    context.launch_configurations.update({
        "environment_collision_avoidance": str(environment).lower(),
        "environment_collision_lidar": str(lidar).lower(),
        "environment_lidar_calibration": str(ROOT / "config/moveit_lidar_calibration.yaml"),
        "use_sim_time": "false",
    })
    container = next(entity for entity in lidar_group.get_sub_entities()
                     if isinstance(entity, ComposableNodeContainer))
    assert (lidar_group.condition.evaluate(context) and container.condition.evaluate(context)) == (environment and lidar)

    # Generate actual Humble component-load requests without loading any node.
    requests = [get_composable_node_load_request(component, context) for component in components]
    mount, cloud = [{p.name: Parameter.from_parameter_msg(p).value for p in request.parameters}
                    for request in requests]
    assert mount["frame_id"] == "body"
    assert mount["child_frame_id"] == cloud["frame_id"]
    assert cloud["max_clouds"] == 1
    assert cloud["fixed_frame_id"] == "odom"
    assert not cloud["circular_buffer"]
    assert not cloud["use_sim_time"]
    assert "cloud:=/velodyne/points" in requests[1].remap_rules
    assert "assembled_cloud:=/moveit_environment/lidar/points" in requests[1].remap_rules

    def apply_parameter_actions(entity):
        if entity.condition is not None and not entity.condition.evaluate(context):
            return
        if isinstance(entity, GroupAction):
            for child in entity.get_sub_entities():
                apply_parameter_actions(child)
        elif isinstance(entity, (SetParameter, SetParametersFromFile)):
            entity.execute(context)

    # Only parameter actions run. In particular, no Node or Include is executed.
    apply_parameter_actions(moveit_group)
    parameters = {}
    for entry in context.launch_configurations.get("global_params", []):
        if isinstance(entry, tuple):
            name, value = entry
            parameters[name] = value
        else:
            parameters.update(yaml.safe_load(Path(entry).read_text())["/move_group"]["ros__parameters"])
    if not environment:
        assert parameters == {}
        return
    assert list(parameters["sensors"]) == (["frontleft_depth", "lidar"] if lidar else ["frontleft_depth"])
    assert parameters["frontleft_depth"]["point_cloud_topic"] == "/depth_registered/frontleft/points"
    assert parameters["lidar"]["point_cloud_topic"] == "/moveit_environment/lidar/points"

    arm_defaults = yaml.safe_load((ROOT / "config/arm_motion.yaml").read_text())["/**"]["ros__parameters"]
    assert parameters["frontleft_depth"]["filtered_cloud_topic"] == arm_defaults["arm.environment.filtered_cloud_topic"]
    assert parameters["lidar"]["filtered_cloud_topic"] == arm_defaults["arm.environment.lidar_filtered_cloud_topic"]
