# In fault_detector_spot/behaviour_tree/simple_timer_command.py

from builtin_interfaces.msg import Time
from .simple_command import SimpleCommand
from geometry_msgs.msg import PoseStamped, Quaternion


class ManipulatorMoveCommand(SimpleCommand):
    """
    A SimpleCommand with an extra goal_pose field.
    """

    def __init__(self, command_id: str, stamp: Time, goal_pose: PoseStamped):
        super().__init__(command_id, stamp)
        self.goal_pose = goal_pose

    def __repr__(self):
        ts = f"{self.stamp.sec}.{self.stamp.nanosec:09d}"
        p = self.goal_pose.pose.position
        # if you also want orientation:
        # o = self.goal_pose.pose.orientation
        return (
            f"<PoseCommand id={self.command_id!r} at={ts} "
            f"pose=({p.x:.2f}, {p.y:.2f}, {p.z:.2f})>"
        )
