"""Resolve persistent runtime storage locations."""

import os
from pathlib import Path


def ros_home() -> Path:
    """Return the configured ROS home directory."""
    configured = os.environ.get("ROS_HOME", "").strip()
    if configured:
        return Path(configured).expanduser()
    return Path.home() / ".ros"


def fault_detector_runtime_root() -> Path:
    """Return the persistent fault-detector runtime root."""
    return ros_home() / "fault_detector_spot"


def default_map_root() -> Path:
    """Return the default persistent map storage directory."""
    return fault_detector_runtime_root() / "maps"


def default_recording_root() -> Path:
    """Return the default persistent recording storage directory."""
    return fault_detector_runtime_root() / "recordings"
