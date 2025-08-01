# fault_detector_spot/behaviour_tree/generic_command.py
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, Quaternion
from fault_detector_spot.behaviour_tree.simple_command import SimpleCommand


class GenericCommand(SimpleCommand):
    """
    A generic super command that contains every possible field
    from all command types, so it can act as a fallback container.
    """

    def __init__(
            self,
            command_id: str,
            stamp: Time,
            duration: float = None,
            goal_pose: PoseStamped = None,
            tag_id: int = None,
            offset: PoseStamped = None,
            orientation_mode: str = None
    ):
        super().__init__(command_id, stamp)
        self.duration = duration
        self.goal_pose = goal_pose
        self.tag_id = tag_id
        self.offset = offset
        self.orientation_mode = orientation_mode

    def __repr__(self):
        ts = f"{self.stamp.sec}.{self.stamp.nanosec:09d}"
        parts = [f"id={self.command_id!r}", f"at={ts}"]

        if self.duration is not None:
            parts.append(f"duration={self.duration}s")
        if self.tag_id is not None:
            parts.append(f"tag_id={self.tag_id}")
        if self.orientation_mode:
            parts.append(f"orientation={self.orientation_mode}")
        if self.goal_pose:
            p = self.goal_pose.pose.position
            parts.append(f"pose=({p.x:.2f}, {p.y:.2f}, {p.z:.2f})")

        return "<GenericCommand " + " ".join(parts) + ">"
