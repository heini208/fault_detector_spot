"""Tests for persistent runtime storage composition."""

import inspect
from pathlib import Path

from fault_detector_spot.application.behaviour_tree.behaviours.helper_initializer import (
    HelperInitializer,
)
from fault_detector_spot.application.recording.record_manager_node import (
    RecordManager,
)
from fault_detector_spot.mapping.runtime.rtab_helper import RTABHelper
from fault_detector_spot.shared.persistence import runtime_paths


def test_runtime_paths_default_under_ros_home(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_HOME", str(tmp_path))

    assert runtime_paths.default_map_root() == (
        tmp_path / "fault_detector_spot" / "maps"
    )
    assert runtime_paths.default_recording_root() == (
        tmp_path / "fault_detector_spot" / "recordings"
    )


def test_runtime_paths_default_to_dot_ros(monkeypatch, tmp_path):
    monkeypatch.delenv("ROS_HOME", raising=False)
    monkeypatch.setattr(Path, "home", lambda: tmp_path)

    assert runtime_paths.ros_home() == tmp_path / ".ros"


def test_rtab_helper_accepts_explicit_map_root():
    signature = inspect.signature(RTABHelper.__init__)

    assert "maps_dir" in signature.parameters
    source = inspect.getsource(RTABHelper.__init__)
    assert "default_map_root()" in source
    assert "get_package_share_directory" not in source


def test_helper_initializer_forwards_navigation_map_root():
    source = inspect.getsource(HelperInitializer.setup)

    assert '"navigation.map_root"' in source
    assert "maps_dir=map_root" in source


def test_record_manager_uses_runtime_parameter_not_package_share():
    source = inspect.getsource(RecordManager.__init__)

    assert '"recording.root"' in source
    assert "default_recording_root()" in source
    assert "get_package_share_directory" not in source


def test_top_level_launch_has_shared_runtime_root_arguments():
    source = (
        Path(__file__).parents[1]
        / "launch"
        / "fault_detector_launch.py"
    ).read_text(encoding="utf-8")

    assert '"navigation_map_root"' in source
    assert '"recording_root"' in source
    assert source.count('"navigation.map_root": navigation_map_root') == 2
    assert '"recording.root": recording_root' in source
