#!/usr/bin/env python3

from std_msgs.msg import Header

class SimpleCommand:
    """
    A tiny container for a single command:
      - id:    the command ID string (e.g. "stow_arm")
      - stamp: a std_msgs/Header carrying the time it was received

    Usage:
      cmd = SimpleCommand("stow_arm", node.get_clock().now().to_msg())
    """
    def __init__(self, command_id: str, stamp: Header):
        self.id = command_id
        self.stamp = stamp

    def __repr__(self):
        ts = f"{self.stamp.sec}.{self.stamp.nanosec:09d}"
        return f"<SimpleCommand id={self.id!r} at={ts}>"
