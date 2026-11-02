"""Pure imports and root test collection must not load ROS or Qt."""

from pathlib import Path
import subprocess
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize("modules", [
    (
        "fault_detector_spot.application.controllers.command_controller",
        "fault_detector_spot.application.controllers.sensor_attachment_controller",
        "fault_detector_spot.application.controllers.sensor_registry_controller",
    ),
    (
        "fault_detector_spot.shared.geometry.models",
        "fault_detector_spot.shared.geometry.rotation",
        "fault_detector_spot.shared.geometry.movement_frames",
        "fault_detector_spot.inspection.geometry.surface_plane",
        "fault_detector_spot.inspection.model.sensor_geometry",
    ),
    (
        "fault_detector_spot.shared.persistence.basic_settings",
        "fault_detector_spot.shared.persistence.advanced_settings",
        "fault_detector_spot.shared.persistence.settings_parameter_files",
    ),
    (
        "fault_detector_spot.shared.persistence.file_storage",
        "fault_detector_spot.application.recording.recording_repository",
        "fault_detector_spot.inspection.repository.object_repository",
        "fault_detector_spot.inspection.repository.sensor_repository",
        "fault_detector_spot.inspection.measurement.measurement_repository",
    ),
], ids=["controllers", "geometry", "settings", "persistence"])
def test_pure_imports_without_ros_or_qt(modules):
    result = subprocess.run(
        [sys.executable, "-I", "-c", """
import importlib
from importlib.abc import MetaPathFinder
from pathlib import Path
import runpy
import sys

class NoRosOrQt(MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        root = fullname.partition('.')[0]
        if (
            root.endswith(('_msgs', '_interfaces'))
            or root.startswith(('PyQt', 'PySide', 'tf2', 'rosidl'))
            or root in {'rclpy', 'ament_index_python', 'launch', 'launch_ros',
                        'synchros2', 'py_trees_ros'}
        ):
            raise ModuleNotFoundError(f'ROS/Qt dependency is unavailable: {fullname}')

sys.meta_path.insert(0, NoRosOrQt())
root = Path(sys.argv[1])
sys.path.insert(0, str(root))
runpy.run_path(str(root / 'test' / 'conftest.py'))
for name in sys.argv[2:]:
    module = importlib.import_module(name)
    assert Path(module.__file__).resolve().is_relative_to(root), module.__file__
""", str(ROOT), *modules],
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert result.returncode == 0, result.stdout + result.stderr
