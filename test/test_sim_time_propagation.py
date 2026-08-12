"""Tests for simulated-time propagation into nested runtime launches."""

import inspect

from fault_detector_spot.mapping.runtime.rtab_helper import RTABHelper
from fault_detector_spot.navigation.runtime.nav2_helper import Nav2Helper


class _Parameter:
    def __init__(self, value):
        self.value = value


class _Node:
    def __init__(self, use_sim_time):
        self.use_sim_time = use_sim_time

    def get_parameter(self, name):
        assert name == "use_sim_time"
        return _Parameter(self.use_sim_time)


def test_rtab_helper_reads_ros_use_sim_time_parameter():
    helper = RTABHelper.__new__(RTABHelper)
    helper.node = _Node(True)

    assert helper._use_sim_time()
    assert helper._use_sim_time_launch_arg() == "true"


def test_rtab_helper_defaults_to_wall_time_when_parameter_unavailable():
    helper = RTABHelper.__new__(RTABHelper)
    helper.node = object()

    assert not helper._use_sim_time()
    assert helper._use_sim_time_launch_arg() == "false"


def test_rtab_mapping_and_localization_forward_use_sim_time():
    mapping_source = inspect.getsource(
        RTABHelper.initialize_mapping_from_existing
    )
    localization_source = inspect.getsource(
        RTABHelper.init_localization
    )

    assert "use_sim_time:=" in mapping_source
    assert "_use_sim_time_launch_arg" in mapping_source
    assert "use_sim_time:=" in localization_source
    assert "_use_sim_time_launch_arg" in localization_source


def test_nav2_helper_reads_ros_use_sim_time_parameter():
    helper = Nav2Helper.__new__(Nav2Helper)
    helper.node = _Node(True)

    assert helper._use_sim_time()
    assert helper._use_sim_time_launch_arg() == "true"


def test_nav2_no_longer_reads_nonexistent_node_attribute():
    source = inspect.getsource(Nav2Helper.start)

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
