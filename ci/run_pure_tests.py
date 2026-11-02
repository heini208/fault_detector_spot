#!/usr/bin/env python3
"""Run the explicit pure test lane with ROS and Qt imports forbidden."""

from importlib.abc import MetaPathFinder
import os
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / "ci" / "pure-tests.txt"


def is_runtime_dependency(module_name):
    """Identify ROS packages, generated interfaces, and Qt bindings."""
    root = module_name.partition(".")[0]
    return (
        root.endswith(("_msgs", "_interfaces"))
        or root.startswith(("PyQt", "PySide", "tf2", "rosidl", "ament_", "ros2"))
        or root in {
            "rclpy", "rclcpp", "rospy", "rosgraph", "launch", "launch_ros",
            "synchros2", "py_trees_ros",
        }
    )


class NoRosOrQt(MetaPathFinder):
    """Fail forbidden imports even when a test uses pytest.importorskip."""

    def find_spec(self, fullname, path=None, target=None):
        if is_runtime_dependency(fullname):
            raise RuntimeError(f"Pure test lane forbids ROS/Qt dependency: {fullname}")
        return None


def main():
    """Resolve the allowlist from the checkout and pass options to pytest."""
    tests = [
        str(ROOT / line.strip())
        for line in MANIFEST.read_text(encoding="utf-8").splitlines()
        if line.strip() and not line.lstrip().startswith("#")
    ]
    sys.dont_write_bytecode = True
    os.environ["PYTEST_DISABLE_PLUGIN_AUTOLOAD"] = "1"
    sys.path.insert(0, str(ROOT))
    for name in tuple(sys.modules):
        if is_runtime_dependency(name):
            raise RuntimeError(f"Pure test lane already loaded ROS/Qt dependency: {name}")
    sys.meta_path.insert(0, NoRosOrQt())

    import pytest

    return pytest.main(["--rootdir", str(ROOT), *tests, *sys.argv[1:]])


if __name__ == "__main__":
    raise SystemExit(main())
