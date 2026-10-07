"""Keep lidar height handling coherent in both mapping and localization.

Preserving odometry Z alone is insufficient: absolute-map height filtering would
then discard nearby obstacles when Spot's odometry origin is above the cutoff.
This coupled launch contract must hold in both runtime modes.
"""

import importlib.util
from pathlib import Path

import ament_index_python.packages
import pytest


@pytest.mark.parametrize("incremental_memory", ["true", "false"])
def test_planar_lidar_mapping_preserves_height_and_uses_local_height_limits(
    monkeypatch, incremental_memory,
):
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
    node = launch_file.make_rtabmap_node(incremental_memory, condition=None)
    parameters = node["parameters"][0]

    assert parameters["Mem/IncrementalMemory"] == incremental_memory
    assert parameters["Reg/Force3DoF"] == "true"
    assert parameters["Grid/3D"] == "true"
    assert parameters["RGBD/ForceOdom3DoF"] == "false"
    assert parameters["Grid/MapFrameProjection"] == "false"
    assert 0 < float(parameters["Grid/MaxGroundHeight"]) < float(
        parameters["Grid/MaxObstacleHeight"]
    )
