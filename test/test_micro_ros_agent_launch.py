"""Contract tests for the system-managed micro-ROS Agent."""

from pathlib import Path


ROOT = Path(__file__).parents[1]


def test_primary_launch_manages_configurable_micro_ros_agent():
    source = (
        ROOT / "launch" / "fault_detector_launch.py"
    ).read_text(encoding="utf-8")

    assert 'package="micro_ros_agent"' in source
    assert 'executable="micro_ros_agent"' in source
    assert '"launch_micro_ros_agent"' in source
    assert 'default_value="udp4"' in source
    assert 'default_value="8888"' in source
    assert "condition=IfCondition(launch_micro_ros_agent)" in source
    assert "respawn=True" in source
    assert "respawn_delay=2.0" in source


def test_package_declares_micro_ros_agent_runtime_dependency():
    manifest = (ROOT / "package.xml").read_text(encoding="utf-8")

    assert "<exec_depend>micro_ros_agent</exec_depend>" in manifest
