"""Tests for simulated-time propagation into nested runtime launches."""

import inspect

from fault_detector_spot.mapping.runtime.rtabmap_runtime_manager import RtabmapRuntimeManager
from fault_detector_spot.navigation.runtime.nav2_runtime_manager import Nav2RuntimeManager
from fault_detector_spot.shared.ros.runtime_manager import RuntimeManager


class _Parameter:
    def __init__(self, value):
        self.value = value


class _Node:
    def __init__(self, use_sim_time):
        self.use_sim_time = use_sim_time

    def get_parameter(self, name):
        assert name == "use_sim_time"
        return _Parameter(self.use_sim_time)


def test_rtabmap_runtime_manager_reads_ros_use_sim_time_parameter():
    helper = RtabmapRuntimeManager.__new__(RtabmapRuntimeManager)
    helper.node = _Node(True)

    assert helper._use_sim_time()
    assert helper._use_sim_time_launch_arg() == "true"


def test_rtabmap_runtime_manager_defaults_to_wall_time_when_parameter_unavailable():
    helper = RtabmapRuntimeManager.__new__(RtabmapRuntimeManager)
    helper.node = object()

    assert not helper._use_sim_time()
    assert helper._use_sim_time_launch_arg() == "false"


def test_both_managers_share_launch_and_sim_time_implementation():
    assert RtabmapRuntimeManager._launch is RuntimeManager._launch
    assert Nav2RuntimeManager._launch is RuntimeManager._launch


def test_nav2_runtime_manager_reads_ros_use_sim_time_parameter():
    helper = Nav2RuntimeManager.__new__(Nav2RuntimeManager)
    helper.node = _Node(True)

    assert helper._use_sim_time()
    assert helper._use_sim_time_launch_arg() == "true"


def test_nav2_no_longer_reads_nonexistent_node_attribute():
    source = inspect.getsource(RuntimeManager._launch)

    assert 'hasattr(self.node, "use_sim_time")' not in source
    assert "_use_sim_time_launch_arg" in source


def test_mapping_launch_applies_use_sim_time_to_all_nested_nodes():
    from pathlib import Path

    path = (
        Path(__file__).parents[1]
        / "launch"
        / "lidar_rtab_mapping_launch.py"
    )
    source = path.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument(\n        "use_sim_time"' in source
    assert source.count(
        '"use_sim_time": LaunchConfiguration("use_sim_time")'
    ) == 3
