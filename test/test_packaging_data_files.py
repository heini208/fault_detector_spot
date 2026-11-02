"""Packaging must ignore obsolete data left in colcon's build directory."""

from pathlib import Path
import runpy
from unittest.mock import patch

import pytest


SOURCE = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize("stale_kind", ["dangling_symlink", "copied_file"])
def test_data_files_follow_source_when_setup_runs_from_build(tmp_path, monkeypatch, stale_kind):
    setup_link = tmp_path / "setup.py"
    setup_link.symlink_to(SOURCE / "setup.py")
    for directory in ("launch", "config"):
        staged = tmp_path / directory
        staged.mkdir()
        for source in (SOURCE / directory).iterdir():
            if source.is_file():
                (staged / source.name).symlink_to(source)
    obsolete = (
        tmp_path / "launch/nav2_spot_launch.py",
        tmp_path / "config/obsolete.yaml",
    )
    for path in obsolete:
        if stale_kind == "dangling_symlink":
            path.symlink_to(tmp_path / "deleted_source" / path.name)
        else:
            path.write_text("obsolete build copy\n")

    monkeypatch.chdir(tmp_path)
    with patch("setuptools.setup") as setup:
        runpy.run_path(str(setup_link))
    files = {
        path for _, paths in setup.call_args.kwargs["data_files"] for path in paths
    }
    assert "launch/fault_detector_launch.py" in files
    assert "launch/nav2_lidar_launch.py" in files
    assert "config/navigate_minimal.xml" in files
    assert "launch/nav2_spot_launch.py" not in files
    assert "config/obsolete.yaml" not in files
    assert all(not Path(path).is_absolute() for path in files)
