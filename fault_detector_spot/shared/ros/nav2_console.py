"""Throttle repeated Nav2 stale-buffer console lines without changing ROS logs."""

import logging
import re
from threading import Lock
import time

from launch.actions import RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.logging import launch_config


_PROCESS_LOGGER = re.compile(
    r"(?P<process>bt_runner|controller_server|planner_server)(?:-\d+)?-(?:stdout|stderr)"
)
_ANSI_SGR = re.compile(r"\x1b\[[0-9;:]*m")
_NUMBER = r"\d+(?:\.\d+)?"
_STALE_BUFFER_WARNING = re.compile(
    r"(?:\[bt_runner(?:-\d+)?\] )?"
    r"(?:\[(?P<child>controller_server|planner_server)(?:-\d+)?\] )?"
    rf"\[WARN\] \[{_NUMBER}\] "
    r"\[(?P<costmap>local_costmap|global_costmap)\.(?P=costmap)\]: "
    r"The /velodyne/(?P<topic>points_obstacles|points_filtered) observation buffer "
    rf"has not been updated for {_NUMBER} seconds, "
    rf"and it should be updated every {_NUMBER} seconds\."
)


class Nav2ObservationWarningConsoleFilter(logging.Filter):
    """Show each costmap/source fault immediately, then at most every 5 seconds."""

    def __init__(self, *, clock=None):
        super().__init__()
        self._clock = clock if clock is not None else time.monotonic
        # The matcher permits only two costmaps and two topics: at most 4 keys.
        self._last_shown = {}
        self._lock = Lock()

    def filter(self, record):
        process = _PROCESS_LOGGER.fullmatch(record.name)
        if process is None:
            return True
        lines = _ANSI_SGR.sub("", record.getMessage()).splitlines()
        if not lines:
            return True
        keys = set()
        for line in lines:
            match = _STALE_BUFFER_WARNING.fullmatch(line)
            if match is None:
                # Never suppress a batch containing any other message.
                return True
            child = match.group("child")
            if process.group("process") == "bt_runner":
                if child is None:
                    return True
            elif child is not None and child != process.group("process"):
                return True
            keys.add((match.group("costmap"), match.group("topic")))
        with self._lock:
            now = self._clock()
            if all(key in self._last_shown and now - self._last_shown[key] < 5.0
                   for key in keys):
                return False
            # A whole record passes if any of its sources needs a reminder.
            for key in keys:
                self._last_shown[key] = now
            return True


def install_nav2_console_throttle(_context):
    """Filter only the outer launch screen handler, removing it at shutdown."""
    handler = launch_config.get_screen_handler()
    if any(isinstance(item, Nav2ObservationWarningConsoleFilter) for item in handler.filters):
        return []
    console_filter = Nav2ObservationWarningConsoleFilter()
    handler.addFilter(console_filter)

    def remove_filter(_event, _context):
        handler.removeFilter(console_filter)

    return [RegisterEventHandler(OnShutdown(on_shutdown=remove_filter))]
