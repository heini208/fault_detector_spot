from builtin_interfaces.msg import Time
from fault_detector_spot.application.behaviour_tree.commands.execution_command import (
    ExecutionCommand,
)


class WaitCommand(ExecutionCommand):
    """
    An execution command with a wait duration in seconds.
    Usage:
      cmd = WaitCommand("wait", node.get_clock().now().to_msg(), duration=3.5)
    """

    def __init__(self, command_id: str, stamp: Time, duration: float):
        super().__init__(command_id, stamp)
        self.duration = duration

    def __repr__(self):
        ts = f"{self.stamp.sec}.{self.stamp.nanosec:09d}"
        return (
            f"<WaitCommand id={self.command_id!r} at={ts}"
            f" duration={self.duration}s>"
        )
