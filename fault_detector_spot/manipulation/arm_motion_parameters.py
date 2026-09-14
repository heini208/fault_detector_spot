"""Arm settings from arm_motion.yaml, with optional ROS parameter overrides."""

from functools import lru_cache
import math
from pathlib import Path

import yaml


def arm_motion_config_path():
    """Use checkout config locally and package share after installation."""
    source = (
        Path(__file__).resolve().parents[2] / "config" / "arm_motion.yaml"
    )
    if source.parent.is_dir():
        return source

    from ament_index_python.packages import get_package_share_directory

    return (
        Path(get_package_share_directory("fault_detector_spot"))
        / "config" / "arm_motion.yaml"
    )


@lru_cache(maxsize=None)
def _load_parameters(path):
    with path.open(encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    try:
        parameters = document["/**"]["ros__parameters"]
    except (TypeError, KeyError) as exception:
        raise ValueError(
            f"{path} must contain /**: ros__parameters"
        ) from exception
    if not isinstance(parameters, dict) or not parameters:
        raise ValueError(f"{path} must contain a non-empty parameter mapping")
    for name, value in parameters.items():
        if not isinstance(name, str) or not name.startswith("arm."):
            raise ValueError(
                f"{path}: expected an arm.* parameter, got {name!r}"
            )
        _validate_value(name, value, value)
    return parameters


def _validate_value(name, value, default):
    if type(default) is float:
        valid = type(value) in (int, float) and math.isfinite(value)
    else:
        valid = (
            type(default) in (bool, int, str)
            and type(value) is type(default)
        )
    if not valid:
        raise ValueError(f"Parameter '{name}' has an invalid value: {value!r}")
    return value


class ArmMotionParameters:
    """Read YAML defaults; declare new YAML keys without a Python registry.

    Explicit constructor values override ROS values, which override YAML.
    YAML is loaded once per process. Edits apply on the next process start.
    """

    def __init__(self, node=None, *, path=None):
        self.path = (
            Path(path) if path is not None else arm_motion_config_path()
        )
        self._defaults = _load_parameters(self.path)
        self._node = node
        if node is not None:
            for name, default in self._defaults.items():
                if not node.has_parameter(name):
                    node.declare_parameter(name, default)

    def get(self, name, override=None):
        """Read an arm-relative key, failing clearly if YAML lacks it."""
        name = "arm." + name
        if name not in self._defaults:
            raise ValueError(f"Missing parameter '{name}' in {self.path}")
        default = self._defaults[name]
        if override is not None:
            value = override
        elif self._node is not None:
            value = self._node.get_parameter(name).value
        else:
            value = default
        return _validate_value(name, value, default)
