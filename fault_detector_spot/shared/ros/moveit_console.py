"""Limit one known MoveIt console warning without changing ROS or file logs."""

import logging
import re
import time

from launch.actions import RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.logging import launch_config


_PROCESS_LOGGER = re.compile(r"move_group(?:-\d+)?-(?:stdout|stderr)")
_NUMBER = r"\d+(?:\.\d+)?"
_STALE_TAG_WARNING = re.compile(
    r"(?:\[move_group(?:-\d+)?\] )?"
    rf"\[WARN\] \[{_NUMBER}\] "
    r"\[moveit_ros\.planning_scene_monitor\.planning_scene_monitor\]: "
    r"Unable to transform object from frame '(?P<tag>tag36h11:\d+)' "
    r"to planning frame'odom' \(Lookup would require extrapolation into the past\. "
    rf" Requested time {_NUMBER} but the earliest data is at time {_NUMBER}, "
    r"when looking up transform from frame \[(?P=tag)\] to frame \[odom\]\)"
)


class MoveItTagWarningConsoleFilter(logging.Filter):
    """Show the first stale hand-tag warning and one reminder every 30 seconds."""

    def __init__(self, *, clock=None):
        super().__init__()
        self._clock = clock if clock is not None else time.monotonic
        self._last_shown = None

    def filter(self, record):
        if not _PROCESS_LOGGER.fullmatch(record.name):
            return True
        lines = record.getMessage().splitlines()
        if not lines or not all(_STALE_TAG_WARNING.fullmatch(line) for line in lines):
            return True
        now = self._clock()
        if self._last_shown is not None and now - self._last_shown < 30.0:
            return False
        self._last_shown = now
        return True


def install_moveit_console_throttle(_context):
    """Install on launch's screen handler only, and remove on launch shutdown."""
    handler = launch_config.get_screen_handler()
    if any(isinstance(item, MoveItTagWarningConsoleFilter) for item in handler.filters):
        return []
    console_filter = MoveItTagWarningConsoleFilter()
    # MoveIt enumerates all TF frames, including expired hand-tag observations.
    # Keep every other PSM warning visible and leave native ROS logs untouched.
    handler.addFilter(console_filter)

    def remove_filter(_event, _context):
        handler.removeFilter(console_filter)

    return [RegisterEventHandler(OnShutdown(on_shutdown=remove_filter))]
