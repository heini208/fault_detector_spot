"""Exercise RTAB-Map's real binary writer against OctoMap's real reader offline.

Source the ROS/workspace environment before running. This optional integration
test compiles only into pytest's temporary directory and starts no ROS nodes.
"""

from array import array
import os
from pathlib import Path
import shutil
import subprocess

import pytest

from fault_detector_spot.manipulation.rtabmap_octomap import validate_binary_occupancy


def test_installed_rtabmap_binary_occupancy_is_compatible_with_octree(tmp_path):
    compiler = shutil.which("g++")
    if compiler is None:
        pytest.skip("g++ is required for the installed-library compatibility check")

    source = Path(__file__).with_suffix(".cpp")
    prefixes = [
        Path(value)
        for variable in ("CMAKE_PREFIX_PATH", "AMENT_PREFIX_PATH")
        for value in os.environ.get(variable, "").split(os.pathsep)
        if value
    ]
    prefixes.extend(
        [source.parents[3] / "install" / "rtabmap", Path("/usr/local"), Path("/usr")]
    )
    headers = [
        header
        for prefix in dict.fromkeys(prefixes)
        for header in prefix.glob("include/rtabmap-*/rtabmap/core/global_map/OctoMap.h")
        if (prefix / "lib" / "librtabmap_core.so").exists()
    ]
    if not headers:
        pytest.skip("Installed RTAB-Map headers/library are required for this check")

    header = headers[0]
    include_root = header.parents[3]
    library_root = include_root.parent.parent / "lib"
    # RTAB-Map's public OctoMap header also includes PCL, Eigen, and OpenCV.
    includes = [include_root]
    for pattern in ("pcl-*", "eigen3", "opencv4"):
        candidates = [
            path
            for base in (Path("/usr/include"), Path("/usr/local/include"))
            for path in base.glob(pattern)
            if path.is_dir()
        ]
        if not candidates:
            pytest.skip(f"Missing {pattern} development headers for RTAB-Map check")
        includes.append(sorted(candidates)[-1])

    binary = tmp_path / "octomap_binary_compatibility"
    environment = os.environ.copy()
    environment["LD_LIBRARY_PATH"] = os.pathsep.join(
        [str(library_root), environment.get("LD_LIBRARY_PATH", "")]
    )
    command = [compiler, "-std=c++17", "-O0", "-Wall", "-Wextra"]
    command.extend(f"-I{path}" for path in includes)
    command.extend(
        [
            str(source),
            f"-L{library_root}",
            f"-Wl,-rpath-link,{library_root}",
            "-lrtabmap_core",
            "-lrtabmap_utilite",
            "-loctomap",
            "-loctomath",
            "-lopencv_core",
            "-o",
            str(binary),
        ]
    )
    compiled = subprocess.run(
        command, env=environment, capture_output=True, text=True, timeout=90, check=False
    )
    assert compiled.returncode == 0, compiled.stdout + compiled.stderr

    fixture = tmp_path / "rtabmap_occupancy.bin"
    result = subprocess.run(
        [str(binary), str(fixture)],
        env=environment,
        capture_output=True,
        text=True,
        timeout=15,
        check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert result.stdout.count("occupied/free/unknown/pruned preserved") == 2
    validate_binary_occupancy(array("b", fixture.read_bytes()))
