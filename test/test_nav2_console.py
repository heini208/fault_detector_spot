"""Exercise launch console records without initializing ROS or starting processes."""

import importlib.util
import io
import logging
from pathlib import Path

from launch import LaunchContext
from launch.actions import OpaqueFunction
from launch.events import Shutdown
from launch.logging import get_output_loggers, handlers
from launch_ros.actions import Node
import pytest

from fault_detector_spot.shared.ros.nav2_console import (
    Nav2ObservationWarningConsoleFilter,
    install_nav2_console_throttle,
)


def warning(costmap="local_costmap", topic="points_obstacles", *, nested=True):
    process = "controller_server-1" if costmap == "local_costmap" else "planner_server-3"
    prefix = "[bt_runner-7] " if nested else ""
    return (
        f"{prefix}[{process}] [WARN] [1792081669.696290553] "
        f"[{costmap}.{costmap}]: The /velodyne/{topic} observation buffer "
        "has not been updated for 2.19 seconds, "
        "and it should be updated every 0.50 seconds."
    )


def record(message=None, name="bt_runner-7-stdout"):
    return logging.LogRecord(name, logging.INFO, "", 0, warning() if message is None else message,
                             (), None)


@pytest.mark.parametrize("colored", [False, True])
def test_console_reminders_preserve_file_logs_and_original_record(tmp_path, colored):
    now = [100.0]
    screen = io.StringIO()
    screen_handler = logging.StreamHandler(screen)
    screen_handler.addFilter(Nav2ObservationWarningConsoleFilter(clock=lambda: now[0]))
    log_path = tmp_path / "nav2.log"
    file_handler = logging.FileHandler(log_path)
    logger = logging.Logger("bt_runner-7-stdout")
    logger.addHandler(screen_handler)
    logger.addHandler(file_handler)
    message = warning()
    if colored:
        message = message.replace("[WARN]", "\x1b[1;33m[WARN]\x1b[m") + "\x1b[0m"
    event = record("%s")
    event.args = (message,)
    original = event.msg, event.args
    try:
        for timestamp in (100.0, 100.1, 104.999, 105.0):
            now[0] = timestamp
            logger.handle(event)
        assert screen.getvalue().splitlines() == [message] * 2
        assert log_path.read_text().splitlines() == [message] * 4
        assert (event.msg, event.args) == original
    finally:
        file_handler.close()


def test_each_costmap_and_topic_has_its_own_bounded_reminder_budget():
    now = [100.0]
    filtering = Nav2ObservationWarningConsoleFilter(clock=lambda: now[0])
    messages = [warning(costmap, topic)
                for costmap in ("local_costmap", "global_costmap")
                for topic in ("points_filtered", "points_obstacles")]
    for message in messages:
        assert filtering.filter(record(message))
        assert not filtering.filter(record(message))
    now[0] = 105.0
    for message in messages:
        assert filtering.filter(record(message))
    assert len(filtering._last_shown) == 4


@pytest.mark.parametrize("message,name", [
    (warning().replace("[WARN]", "[ERROR]"), "bt_runner-7-stdout"),
    (warning().replace("points_obstacles", "other_sensor"), "bt_runner-7-stdout"),
    (warning().replace("local_costmap", "other_costmap"), "bt_runner-7-stdout"),
    (warning().replace("controller_server-1", "other_node-1"), "bt_runner-7-stdout"),
    (warning().replace("[controller_server-1] ", ""), "bt_runner-7-stdout"),
    (warning(), "unrelated-1-stdout"),
    (warning(), "bt_runner-7"),
    (warning(nested=False), "planner_server-3-stdout"),
    (warning() + "\n[ERROR] A different failure", "bt_runner-7-stdout"),
    ("[WARN] Other warning\n" + warning(), "bt_runner-7-stdout"),
    ("\x1b[2K" + warning(), "bt_runner-7-stdout"),
    ("", "bt_runner-7-stdout"),
])
def test_other_messages_and_mixed_records_always_pass_unmodified(message, name):
    filtering = Nav2ObservationWarningConsoleFilter(clock=lambda: 100.0)
    assert filtering.filter(record())
    event = record(message, name)
    assert filtering.filter(event)
    assert filtering.filter(event)
    assert event.getMessage() == message


def test_mixed_known_sources_keep_whole_record_when_one_reminder_is_due():
    now = [100.0]
    filtering = Nav2ObservationWarningConsoleFilter(clock=lambda: now[0])
    assert filtering.filter(record())
    now[0] = 101.0
    both = record(warning() + "\n" + warning("global_costmap"))
    assert filtering.filter(both)
    assert not filtering.filter(both)
    now[0] = 106.0
    assert filtering.filter(both)


def test_launch_output_logger_path_handles_nested_child_and_direct_nav2(monkeypatch):
    handler = handlers.StreamHandler(io.StringIO())
    monkeypatch.setattr(
        "fault_detector_spot.shared.ros.nav2_console.launch_config.get_screen_handler",
        lambda: handler,
    )
    install_nav2_console_throttle(None)
    cases = [
        ("bt_runner-7", warning(), "stdout"),
        ("controller_server-1", warning(nested=False), "stderr"),
        ("planner_server-3", warning("global_costmap", nested=False), "stderr"),
    ]
    for process, message, stream in cases:
        stdout, stderr = get_output_loggers(process, "screen")
        try:
            logger = stdout if stream == "stdout" else stderr
            assert logger.name == f"{process}-{stream}"
            assert handler in logger.handlers
            event = record(message, logger.name)
            # Direct and nested representations share the same costmap budget.
            allowed = handler.filter(event)
            assert allowed == (process != "controller_server-1")
            assert not handler.filter(event)
        finally:
            stdout.removeHandler(handler)
            stderr.removeHandler(handler)


def test_launch_hook_installs_once_and_removes_at_shutdown(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path))
    handler = logging.StreamHandler(io.StringIO())
    monkeypatch.setattr(
        "fault_detector_spot.shared.ros.nav2_console.launch_config.get_screen_handler",
        lambda: handler,
    )
    context = LaunchContext()
    registration, = install_nav2_console_throttle(context)
    assert len(handler.filters) == 1
    assert install_nav2_console_throttle(context) == []
    registration.execute(context)
    event = Shutdown(reason="offline test")
    assert registration.event_handler.matches(event)
    registration.event_handler.handle(event, context)
    assert handler.filters == []


def test_main_launch_installs_throttle_before_starting_application_nodes(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path))
    root = Path(__file__).parents[1]
    spec = importlib.util.spec_from_file_location(
        "fault_detector_nav2_console_launch", root / "launch/fault_detector_launch.py",
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, "get_package_share_directory", lambda _package: str(root))
    monkeypatch.setattr(module, "_moveit_sensor_parameters", lambda *args: [])
    monkeypatch.setattr(module, "install_moveit_console_throttle", lambda _context: [])
    installed = []
    monkeypatch.setattr(module, "install_nav2_console_throttle",
                        lambda _context: installed.append(True) or [])
    context = LaunchContext()
    # Only execute configuration callbacks, never a node or child launch.
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, OpaqueFunction):
            entity.execute(context)
        elif isinstance(entity, Node):
            assert installed == [True]
    assert installed == [True]
