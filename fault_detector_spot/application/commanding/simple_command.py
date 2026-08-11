#!/usr/bin/env python3

from builtin_interfaces.msg import Time

from fault_detector_spot.application.commanding.request_identity import request_id_or_new


class SimpleCommand:
    """
    A tiny container for a single command:
      - id:    the command ID string (e.g. "stow_arm")
      - stamp: a std_msgs/Header carrying the time it was received

    Usage:
      cmd = SimpleCommand("stow_arm", node.get_clock().now().to_msg())
    """

    def __init__(
        self,
        command_id: str,
        stamp: Time,
        request_id: str = "",
    ):
        self.command_id = command_id
        self.stamp = stamp
        self.request_id = request_id_or_new(request_id)

    def __repr__(self):
        ts = f"{self.stamp.sec}.{self.stamp.nanosec:09d}"
        return (
            f"<SimpleCommand id={self.command_id!r} "
            f"request={self.request_id!r} at={ts}>"
        )
