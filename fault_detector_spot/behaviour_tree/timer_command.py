# In fault_detector_spot/behaviour_tree/simple_timer_command.py

from builtin_interfaces.msg import Time
from fault_detector_spot.behaviour_tree.simple_command import SimpleCommand

class TimerCommand(SimpleCommand):
    """
    A SimpleCommand with an extra `duration` field (in seconds).
    Usage:
      cmd = TimerCommand("wait", node.get_clock().now().to_msg(), duration=3.5)
    """

    def __init__(self, command_id: str, stamp: Time, duration: float):
        super().__init__(command_id, stamp)
        self.duration = duration

    def __repr__(self):
        ts = f"{self.stamp.sec}.{self.stamp.nanosec:09d}"
        return (
            f"<TimerCommand id={self.id!r} at={ts}"
            f" duration={self.duration}s>"
        )
