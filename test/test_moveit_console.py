"""Console filtering only; no ROS initialization or child process execution."""

import io
import logging

from launch import LaunchContext
from launch.events import Shutdown
import pytest

from fault_detector_spot.shared.ros.moveit_console import (
    MoveItTagWarningConsoleFilter,
    install_moveit_console_throttle,
)


WARNING = (
    "[move_group-1] [WARN] [1791468983.569475067] "
    "[moveit_ros.planning_scene_monitor.planning_scene_monitor]: "
    "Unable to transform object from frame 'tag36h11:2' to planning frame'odom' "
    "(Lookup would require extrapolation into the past.  "
    "Requested time 1791468973.434852 but the earliest data is at time "
    "1791468973.560739, when looking up transform from frame [tag36h11:2] "
    "to frame [odom])"
)


def record(message=WARNING, name="move_group-1-stderr"):
    # launch logs process output at INFO, independently of the embedded level.
    return logging.LogRecord(name, logging.INFO, "", 0, message, (), None)


def test_screen_throttle_preserves_file_log_and_records(tmp_path):
    now = [100.0]
    screen = io.StringIO()
    screen_handler = logging.StreamHandler(screen)
    screen_handler.addFilter(MoveItTagWarningConsoleFilter(clock=lambda: now[0]))
    log_path = tmp_path / "move_group.log"
    file_handler = logging.FileHandler(log_path)
    logger = logging.Logger("move_group-1-stderr")
    logger.addHandler(screen_handler)
    logger.addHandler(file_handler)
    warning = record("%s")
    warning.args = (WARNING,)
    original = warning.msg, warning.args
    try:
        logger.handle(warning)
        logger.handle(warning)
        now[0] = 129.999
        logger.handle(warning)
        now[0] = 130.0
        logger.handle(warning)
        assert screen.getvalue().splitlines() == [WARNING] * 2
        assert log_path.read_text().splitlines() == [WARNING] * 4
        assert (warning.msg, warning.args) == original
        assert warning.getMessage() == WARNING
    finally:
        file_handler.close()


@pytest.mark.parametrize("message,name", [
    (WARNING.replace("tag36h11:2", "fiducial_276"), "move_group-1-stderr"),
    (WARNING.replace("odom", "map"), "move_group-1-stderr"),
    (WARNING.replace("[WARN]", "[ERROR]"), "move_group-1-stderr"),
    (WARNING.replace("into the past", "into the future"), "move_group-1-stderr"),
    (WARNING.replace("planning_scene_monitor]:", "other]:"), "move_group-1-stderr"),
    (WARNING, "other_move_group-1-stderr"),
    (WARNING, "move_group-1"),
    (WARNING + "\n[ERROR] A different failure", "move_group-1-stderr"),
    ("[WARN] A different warning\n" + WARNING, "move_group-1-stderr"),
    ("", "move_group-1-stderr"),
])
def test_unmatched_messages_always_pass(message, name):
    console_filter = MoveItTagWarningConsoleFilter(clock=lambda: 100.0)
    assert console_filter.filter(record())
    assert console_filter.filter(record(message, name))
    assert console_filter.filter(record(message, name))


def test_one_global_monotonic_throttle_ignores_ros_clock_epoch():
    now = [100.0]
    console_filter = MoveItTagWarningConsoleFilter(clock=lambda: now[0])
    assert console_filter.filter(record())
    replay_warning = record(
        WARNING.replace("1791468983.569475067", "1.000000000")
        .replace("tag36h11:2", "tag36h11:99"),
        "move_group-2-stdout",
    )
    replay_warning.created = 1.0
    now[0] = 101.0
    assert not console_filter.filter(replay_warning)
    now[0] = 130.0
    assert console_filter.filter(replay_warning)
    assert not console_filter.filter(record(WARNING + "\n" + WARNING))


def test_launch_hook_installs_once_and_removes_at_shutdown(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path))
    handler = logging.StreamHandler(io.StringIO())
    monkeypatch.setattr(
        "fault_detector_spot.shared.ros.moveit_console.launch_config.get_screen_handler",
        lambda: handler,
    )
    context = LaunchContext()
    registration, = install_moveit_console_throttle(context)
    assert len(handler.filters) == 1
    assert install_moveit_console_throttle(context) == []
    assert len(handler.filters) == 1
    registration.execute(context)
    event = Shutdown(reason="offline test")
    assert registration.event_handler.matches(event)
    registration.event_handler.handle(event, context)
    assert handler.filters == []
    assert len(install_moveit_console_throttle(context)) == 1
