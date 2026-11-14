"""Keep lidar height handling coherent in both mapping and localization.

Preserving odometry Z alone is insufficient: absolute-map height filtering would
then discard nearby obstacles when Spot's odometry origin is above the cutoff.
This coupled launch contract must hold in both runtime modes.
"""

import importlib.util
from pathlib import Path

import ament_index_python.packages
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from rclpy.qos import ReliabilityPolicy
import pytest
import yaml


@pytest.fixture
def launch_file(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path))
    root = Path(__file__).parents[1]
    monkeypatch.setattr(
        ament_index_python.packages, "get_package_share_directory", lambda _: str(root),
    )
    spec = importlib.util.spec_from_file_location(
        "lidar_mapping_launch", root / "launch" / "lidar_rtab_mapping_launch.py",
    )
    launch_file = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch_file)
    # Inspect the real factory's arguments without starting a process or ROS node.
    monkeypatch.setattr(launch_file, "Node", lambda **kwargs: kwargs)
    return launch_file


def node_parameters(launch_file, node):
    context = LaunchContext()
    for entity in launch_file.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    parameter_file, overrides = node["parameters"]
    path = Path(parameter_file.perform(context))
    assert path == Path(launch_file.pkg) / "config" / "lidar_mapping.yaml"
    parameters = yaml.safe_load(path.read_text())[node["name"]]["ros__parameters"]
    if node["name"] == "rtabmap":
        assert all(isinstance(value, str) for key, value in parameters.items() if "/" in key)
    return {**parameters, **overrides}


@pytest.mark.parametrize("incremental_memory", ["true", "false"])
@pytest.mark.parametrize("ground_height", ["0.05", "0.12"])
def test_planar_lidar_mapping_preserves_height_and_uses_local_height_limits(
    launch_file, incremental_memory, ground_height,
):
    node = launch_file.make_rtabmap_node(incremental_memory, condition=None)
    parameters = node_parameters(launch_file, node)
    context = LaunchContext()
    context.launch_configurations["ground_max_height"] = ground_height

    assert parameters["Mem/IncrementalMemory"] == incremental_memory
    assert parameters["Reg/Force3DoF"] == "true"
    assert parameters["Grid/3D"] == "true"
    assert parameters["RGBD/ForceOdom3DoF"] == "false"
    assert parameters["Grid/MapFrameProjection"] == "false"
    assert parameters["pub_loc_pose_only_when_localizing"] is False
    assert parameters["Grid/MaxGroundHeight"].evaluate(context) == ground_height
    assert 0 < float(ground_height) < float(
        parameters["Grid/MaxObstacleHeight"]
    )


@pytest.mark.parametrize("incremental_memory", ["true", "false"])
def test_scan_only_proximity_registration_has_an_odometry_guess(launch_file, incremental_memory):
    node = launch_file.make_rtabmap_node(incremental_memory, condition=None)
    parameters = node_parameters(launch_file, node)
    assert parameters["subscribe_scan_cloud"] is True
    assert parameters["Reg/Strategy"] == "1"
    # Admission must remain sound even if a custom parameter file changes it.
    assert node["parameters"][1]["Reg/Strategy"] == "1"
    assert parameters["RGBD/ProximityBySpace"] == "true"
    assert parameters["RGBD/ProximityOdomGuess"] == "true"


def test_standalone_defaults_preserve_legacy_maps_without_moving_nav2_ground_gate(launch_file):
    entities = launch_file.generate_launch_description().entities
    context = LaunchContext()
    for entity in entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    assert context.launch_configurations["frame_id"] == "base_link"
    assert context.launch_configurations["body_frame"] == "base_link"
    assert context.launch_configurations["ground_frame"] == "feet_center"
    assert context.launch_configurations["ground_max_height"] == "0.12"
    filtering_node = next(
        entity for entity in entities
        if isinstance(entity, dict) and entity["name"] == "lidar_self_filter"
    )
    filtering = node_parameters(launch_file, filtering_node)
    assert filtering["base_frame"].perform(context) == "base_link"
    assert filtering["ground_frame"].perform(context) == "feet_center"
    assert filtering["minimum_obstacle_height"] == 0.05
    # New map metadata opts into feet-frame RTAB mapping explicitly. Neither
    # RTAB choice changes the body-relative arm box or ground-relative Nav2 gate.
    context.launch_configurations["frame_id"] = "feet_center"
    context.launch_configurations["ground_max_height"] = "0.05"
    assert filtering["base_frame"].perform(context) == "base_link"
    assert filtering["ground_frame"].perform(context) == "feet_center"


@pytest.mark.parametrize("incremental_memory", ["true", "false"])
def test_rtabmap_keeps_latest_scan_and_resolves_odometry_from_capture_time_tf(
    launch_file, incremental_memory,
):
    parameters = node_parameters(
        launch_file, launch_file.make_rtabmap_node(incremental_memory, condition=None),
    )
    # With one scan stream and TF odometry there is no cross-topic synchronizer
    # requiring queued old scans. DDS history must also remain bounded during ICP.
    assert parameters["subscribe_scan_cloud"] is True
    assert parameters["subscribe_scan"] is False
    assert parameters["subscribe_odom"] is False
    assert parameters["topic_queue_size"] == 1
    assert parameters["sync_queue_size"] == 1
    assert parameters["qos_scan"] == int(ReliabilityPolicy.BEST_EFFORT)
    assert parameters["Rtabmap/DetectionRate"] == "2.0"


def test_rtabmap_timing_verbosity_is_scoped_and_can_be_reenabled(launch_file):
    context = LaunchContext()
    for entity in launch_file.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    assert context.launch_configurations["rtabmap_log_level"] == "warn"
    for mode in ("true", "false"):
        arguments = launch_file.make_rtabmap_node(mode, condition=None)["arguments"]
        assert arguments[:2] == ["--ros-args", "--log-level"]
        logger_level = normalize_to_list_of_substitutions(arguments[2])
        assert perform_substitutions(context, logger_level) == "rtabmap:=warn"
        context.launch_configurations["rtabmap_log_level"] = "info"
        assert perform_substitutions(context, logger_level) == "rtabmap:=info"
        context.launch_configurations["rtabmap_log_level"] = "warn"


@pytest.mark.parametrize(
    "incremental_memory,expected_neighbors,expected_iterations",
    [("true", "1", "20"), ("false", "5", "10")],
)
def test_localization_icp_tuning_does_not_change_mapping_parameters(
    launch_file, incremental_memory, expected_neighbors, expected_iterations,
):
    parameters = node_parameters(
        launch_file, launch_file.make_rtabmap_node(incremental_memory, condition=None),
    )
    assert parameters["RGBD/ProximityPathMaxNeighbors"] == expected_neighbors
    assert parameters["Icp/Iterations"] == expected_iterations
    assert parameters["RGBD/ProximityGlobalScanMap"] == "false"
    assert parameters["Icp/VoxelSize"] == "0.10"
    assert parameters["Icp/PointToPlaneK"] == "20"
    assert parameters["Icp/MaxCorrespondenceDistance"] == "1.0"
