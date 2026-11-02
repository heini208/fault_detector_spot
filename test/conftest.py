"""Isolate persistent test state without importing ROS or Qt.

Executor-specific guard adaptation lives in test_arm_movement_executor.py so
collecting pure domain tests does not import the execution stack.
"""

import os
import sys
from tempfile import TemporaryDirectory

import pytest


def _clear_startup_settings():
    module = sys.modules.get(
        'fault_detector_spot.shared.persistence.basic_settings'
    )
    if module is not None:
        module._startup_settings.cache_clear()


@pytest.hookimpl(tryfirst=True)
def pytest_configure(config):
    """Protect collection and session fixtures from user runtime files."""
    runtime_directory = TemporaryDirectory(prefix='fault-detector-tests-')
    original_ros_home = os.environ.get('ROS_HOME')
    os.environ['ROS_HOME'] = runtime_directory.name
    _clear_startup_settings()

    def restore_runtime_home():
        if original_ros_home is None:
            os.environ.pop('ROS_HOME', None)
        else:
            os.environ['ROS_HOME'] = original_ros_home
        _clear_startup_settings()
        runtime_directory.cleanup()

    config.add_cleanup(restore_runtime_home)


@pytest.fixture(autouse=True)
def isolated_runtime_settings(monkeypatch, tmp_path):
    """Give each test and its subprocesses fresh runtime storage."""
    monkeypatch.setenv('ROS_HOME', str(tmp_path / 'ros'))
    _clear_startup_settings()
    yield
    _clear_startup_settings()
